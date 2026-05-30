//! Parsing of the FAT (file allocation table): the header, and the packed tree
//! of nodes that indexes every file in the archive.
//!
//! All multi-byte fields are little-endian, and the node layout below is the
//! 64-bit PC layout (`archive_platform_pc` → `platform_pointer_64bit`). Since
//! this tool only targets little-endian hosts, fields are read directly with
//! [`bytemuck`] (native-endian); a big-endian archive (PS3/Xbox360) would need
//! byte-swapping, which the header's `endian_string` would flag.

use std::fs::File;
use std::io::{self, Read, Seek, SeekFrom};
use std::path::Path;

use bytemuck::{Pod, Zeroable};

// The FAT stores 64-bit offsets; we read them as u64 and use them as usize for
// indexing the buffer and seeking the file. That truncates on a 32-bit target,
// so require usize == u64 (a 64-bit host).
const _: () = assert!(std::mem::size_of::<usize>() == std::mem::size_of::<u64>());

/// On-disk FAT header — `sources/vostok/vfs/sources/fat_header.h`.
///
/// `fat_header` has no `#pragma pack`, so the two `u32`s are 4-byte aligned and
/// there are 2 padding bytes after the 14-byte endian string (24 bytes total).
/// The header is followed by `buffer_size` bytes of packed node buffer, then the
/// data blob.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct FatHeader {
    endian_string: [u8; 14],
    _pad: [u8; 2],
    num_nodes: u32,
    buffer_size: u32,
}

/// File-payload location — `archive_file_node_base` in
/// `sources/vostok/vfs/sources/archive_file_node_base.h`.
///
/// `pos_in_db` is a `file_size_type` (u64 on 64-bit) and the 4 bytes after
/// `size_in_db` are alignment padding before it.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct ArchiveFileNodeBase {
    /// Bytes stored in the db (compressed size when compressed).
    size_in_db: u32,
    _pad: u32,
    /// Absolute file offset of the payload.
    pos_in_db: u64,
    hash: u32,
    // Trailing padding to the struct's 8-byte alignment (pack(8) makes the C++
    // struct 24 bytes); explicit so bytemuck::Pod accepts it.
    _pad2: u32,
}

bitflags::bitflags! {
    /// `vfs_node_enum` — `sources/vostok/vfs/base_node.h`. The node's concrete
    /// class (and thus its byte layout) is selected by these flags.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub struct NodeFlags: u16 {
        const FOLDER           = 1 << 0;
        const PHYSICAL         = 1 << 1;
        const ARCHIVE          = 1 << 2;
        const MOUNT_ROOT       = 1 << 3;
        const COMPRESSED       = 1 << 4;
        const REPLICATED       = 1 << 5;
        const INLINED          = 1 << 6;
        const SUB_FAT          = 1 << 7;
        const SOFT_LINK        = 1 << 8;
        const HARD_LINK        = 1 << 9;
        const MOUNT_HELPER     = 1 << 10;
        const ERASED           = 1 << 11;
        const EXTERNAL_SUB_FAT = 1 << 12;
        const UNIVERSAL_FILE   = 1 << 13;
        const MARKED_TO_UNLINK = 1 << 14;
    }
}

// ---- 64-bit struct member offsets (verified via probe against MSVC pack(8)) -
// base_node<64>: pointers are 8 bytes; m_flags is a u16, m_name is inline.
const BASE_NODE_NEXT: usize = 24;
const BASE_NODE_FLAGS: usize = 48;
const BASE_NODE_NAME: usize = 51;

// base_off = offset of the `base_node` member inside each final node class.
// (A stored node pointer points AT the base_node; we subtract base_off to find
//  the start of the final class where the archive_file_node_base / inline data
//  fields live.)
const BASE_OFF_FOLDER: usize = 16;
const BASE_OFF_FILE: usize = 24; // archive_file_node : afnb; base
const BASE_OFF_COMPRESSED: usize = 32; // afnb; u32 ucs; base
const BASE_OFF_INLINE: usize = 40; // afnb; aifnb; base
const BASE_OFF_INLINE_COMPRESSED: usize = 48; // afnb; aifnb; u32 ucs; base
const BASE_OFF_SOFT_LINK: usize = 8;
const BASE_OFF_HARD_LINK: usize = 8;
const BASE_OFF_ERASED: usize = 0;
const BASE_OFF_EXTERNAL: usize = 16;
// archive_folder_mount_root_node<64>: mount_root_node_base(104) + 3*char[260]
// + char[32] + 2*ptr(16) => folder@936, folder.base@936+16 = 952.
const BASE_OFF_MOUNT_ROOT: usize = 952;

// archive_inline_file_node_base: ptr m_inlined_data @0; u32 m_inlined_size @8
// (relative to the start of the aifnb sub-object, which immediately follows the
//  24-byte afnb sub-object).
const AIFNB_OFF: usize = 24; // offset of aifnb sub-object from node start

// mount_root_node_base: the `node` pointer field is the 9th pointer (offset 64);
// it stores the buffer offset of the root folder's base_node.
const MOUNT_ROOT_NODE_PTR_OFF: usize = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeKind {
    Folder,
    File { compressed: bool, inlined: bool },
    SoftLink,
    HardLink,
    Erased,
    External,
    Other,
}

#[derive(Debug, Clone)]
pub struct FileEntry {
    /// Full virtual path relative to the archive root (e.g. `vostok/foo.cfg`).
    pub path: String,
    pub kind: NodeKind,
    /// Number of bytes stored in the db (compressed size when compressed).
    pub size_in_db: u32,
    /// Uncompressed size (== size_in_db for uncompressed files).
    pub uncompressed_size: u32,
    /// Absolute file offset of the payload (for non-inline files).
    pub pos_in_db: u64,
    /// For inline files: offset of the payload inside the FAT buffer.
    pub inline_buffer_offset: Option<u64>,
}

pub struct Archive {
    file: File,
    /// The FAT node buffer (buffer_size bytes), loaded fully into memory.
    buf: Vec<u8>,
    pub num_nodes: u32,
    pub buffer_size: u32,
}

impl Archive {
    pub fn open<P: AsRef<Path>>(path: P) -> io::Result<Archive> {
        let mut file = File::open(path)?;
        let mut header_bytes = [0u8; std::mem::size_of::<FatHeader>()];
        file.read_exact(&mut header_bytes)?;
        let header: FatHeader = bytemuck::pod_read_unaligned(&header_bytes);

        if &header.endian_string[0..13] != b"little-endian" {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "unexpected endian string: {:?}",
                    String::from_utf8_lossy(&header.endian_string)
                ),
            ));
        }

        let mut buf = vec![0u8; header.buffer_size as usize];
        file.read_exact(&mut buf)?;

        Ok(Archive {
            file,
            buf,
            num_nodes: header.num_nodes,
            buffer_size: header.buffer_size,
        })
    }

    /// Read a little-endian POD value from the FAT buffer at `off`.
    fn read<T: Pod>(&self, off: usize) -> T {
        bytemuck::pod_read_unaligned(&self.buf[off..off + std::mem::size_of::<T>()])
    }

    fn flags_at(&self, base_node_off: usize) -> NodeFlags {
        NodeFlags::from_bits_retain(self.read::<u16>(base_node_off + BASE_NODE_FLAGS))
    }

    fn name_at(&self, base_node_off: usize) -> String {
        let start = base_node_off + BASE_NODE_NAME;
        let mut end = start;
        while end < self.buf.len() && self.buf[end] != 0 {
            end += 1;
        }
        String::from_utf8_lossy(&self.buf[start..end]).into_owned()
    }

    /// Offset (in the FAT buffer) of the root folder's base_node.
    fn root_base_node_off(&self) -> usize {
        // The mount root node sits at buffer offset 0; its `node` field points
        // at the root folder's base_node. Trust that field, falling back to the
        // computed mount-root layout offset.
        let ptr = self.read::<u64>(MOUNT_ROOT_NODE_PTR_OFF) as usize;
        if ptr != 0 && ptr < self.buf.len() {
            ptr
        } else {
            BASE_OFF_MOUNT_ROOT
        }
    }

    fn kind_for(&self, flags: NodeFlags) -> NodeKind {
        if flags.contains(NodeFlags::FOLDER) {
            NodeKind::Folder
        } else if flags.contains(NodeFlags::SOFT_LINK) {
            NodeKind::SoftLink
        } else if flags.contains(NodeFlags::HARD_LINK) {
            NodeKind::HardLink
        } else if flags.contains(NodeFlags::ERASED) {
            NodeKind::Erased
        } else if flags.contains(NodeFlags::EXTERNAL_SUB_FAT) {
            NodeKind::External
        } else if flags.contains(NodeFlags::ARCHIVE) {
            NodeKind::File {
                compressed: flags.contains(NodeFlags::COMPRESSED),
                inlined: flags.contains(NodeFlags::INLINED),
            }
        } else {
            NodeKind::Other
        }
    }

    /// base_off (offset of base_node within the final node class) for `flags`.
    fn base_off_for(&self, flags: NodeFlags) -> usize {
        if flags.contains(NodeFlags::MOUNT_ROOT) {
            BASE_OFF_MOUNT_ROOT
        } else if flags.contains(NodeFlags::FOLDER) {
            BASE_OFF_FOLDER
        } else if flags.contains(NodeFlags::SOFT_LINK) {
            BASE_OFF_SOFT_LINK
        } else if flags.contains(NodeFlags::HARD_LINK) {
            BASE_OFF_HARD_LINK
        } else if flags.contains(NodeFlags::ERASED) {
            BASE_OFF_ERASED
        } else if flags.contains(NodeFlags::EXTERNAL_SUB_FAT) {
            BASE_OFF_EXTERNAL
        } else if flags.contains(NodeFlags::INLINED) {
            if flags.contains(NodeFlags::COMPRESSED) {
                BASE_OFF_INLINE_COMPRESSED
            } else {
                BASE_OFF_INLINE
            }
        } else if flags.contains(NodeFlags::COMPRESSED) {
            BASE_OFF_COMPRESSED
        } else {
            BASE_OFF_FILE
        }
    }

    /// Build a flat list of every file entry, reconstructing virtual paths by
    /// walking the folder/child tree.
    pub fn list(&self) -> Vec<FileEntry> {
        let mut out = Vec::new();
        let root = self.root_base_node_off();
        // The root mount-root node has an empty name; descend into its children.
        debug_assert!(self.flags_at(root).contains(NodeFlags::FOLDER));
        self.walk_folder_children(root, "", &mut out);
        out
    }

    fn first_child_of_folder(&self, base_node_off: usize, flags: NodeFlags) -> usize {
        // base_folder_node: m_first_child is at (base_node - 16) for normal
        // folders, and at (folder.base - 16) for the mount root too (folder
        // is a base_folder_node embedded in the mount root).
        let folder_start = base_node_off - self.base_off_for(flags) + folder_start_within(flags);
        self.read::<u64>(folder_start) as usize
    }

    fn walk_folder_children(
        &self,
        folder_base_node_off: usize,
        prefix: &str,
        out: &mut Vec<FileEntry>,
    ) {
        let flags = self.flags_at(folder_base_node_off);
        let mut child = self.first_child_of_folder(folder_base_node_off, flags);
        while child != 0 {
            self.visit_node(child, prefix, out);
            child = self.read::<u64>(child + BASE_NODE_NEXT) as usize;
        }
    }

    fn visit_node(&self, base_node_off: usize, prefix: &str, out: &mut Vec<FileEntry>) {
        let flags = self.flags_at(base_node_off);
        let name = self.name_at(base_node_off);
        let path = if prefix.is_empty() {
            name.clone()
        } else {
            format!("{}/{}", prefix, name)
        };
        let kind = self.kind_for(flags);

        match kind {
            NodeKind::Folder => {
                self.walk_folder_children(base_node_off, &path, out);
            }
            NodeKind::File { .. } => {
                out.push(self.read_file_fields(base_node_off, flags, path));
            }
            NodeKind::HardLink => {
                // hard_link_node: referenced (node pointer) at node_start+0,
                // base_node at node_start+8. The referenced offset points at
                // the referenced node's base_node; resolve it and copy its
                // payload location, but keep this node's own path.
                let node_start = base_node_off - BASE_OFF_HARD_LINK;
                let ref_base_off = self.read::<u64>(node_start) as usize;
                if ref_base_off != 0 && ref_base_off < self.buf.len() {
                    let ref_flags = self.flags_at(ref_base_off);
                    let mut entry = self.read_file_fields(ref_base_off, ref_flags, path);
                    // Keep the referenced file kind so extraction/decompression
                    // works for the resolved target.
                    entry.kind = self.kind_for(ref_flags);
                    out.push(entry);
                } else {
                    out.push(FileEntry {
                        path,
                        kind,
                        size_in_db: 0,
                        uncompressed_size: 0,
                        pos_in_db: 0,
                        inline_buffer_offset: None,
                    });
                }
            }
            // Soft-links / erased / external nodes do not carry directly
            // extractable payloads; record them as zero-size entries.
            _ => {
                out.push(FileEntry {
                    path,
                    kind,
                    size_in_db: 0,
                    uncompressed_size: 0,
                    pos_in_db: 0,
                    inline_buffer_offset: None,
                });
            }
        }
    }

    /// Read the file-payload fields for a file node given its base_node offset.
    fn read_file_fields(&self, base_node_off: usize, flags: NodeFlags, path: String) -> FileEntry {
        let compressed = flags.contains(NodeFlags::COMPRESSED);
        let inlined = flags.contains(NodeFlags::INLINED);
        let node_start = base_node_off - self.base_off_for(flags);
        // archive_file_node_base sits at the start of every file node class.
        let afnb: ArchiveFileNodeBase = self.read(node_start);
        let uncompressed_size = if compressed {
            // archive(_inline)_compressed_file_node: u32 uncompressed_size,
            // located right before `base`.
            self.read::<u32>(base_node_off - 8)
        } else {
            afnb.size_in_db
        };
        let inline_buffer_offset = if inlined {
            // aifnb.m_inlined_data holds the buffer offset of the data.
            Some(self.read::<u64>(node_start + AIFNB_OFF))
        } else {
            None
        };
        FileEntry {
            path,
            kind: NodeKind::File {
                compressed,
                inlined,
            },
            size_in_db: afnb.size_in_db,
            uncompressed_size,
            pos_in_db: afnb.pos_in_db,
            inline_buffer_offset,
        }
    }

    /// Read the raw (possibly compressed) payload bytes for a file entry.
    pub fn read_raw(&mut self, e: &FileEntry) -> io::Result<Vec<u8>> {
        if let Some(buf_off) = e.inline_buffer_offset {
            let start = buf_off as usize;
            let end = start + e.size_in_db as usize;
            return Ok(self.buf[start..end].to_vec());
        }
        let mut data = vec![0u8; e.size_in_db as usize];
        self.file.seek(SeekFrom::Start(e.pos_in_db))?;
        self.file.read_exact(&mut data)?;
        Ok(data)
    }

    /// Read the payload for a file entry.
    ///
    /// The shipped resources.db stores everything uncompressed. PPMd-compressed
    /// payloads are not handled here — the decoder lives on the `ppmd` branch;
    /// this returns an error for them.
    pub fn read_file(&mut self, e: &FileEntry) -> io::Result<Vec<u8>> {
        let raw = self.read_raw(e)?;
        match e.kind {
            NodeKind::File {
                compressed: true, ..
            } => Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "PPMd-compressed payload — decoder is on the `ppmd` branch",
            )),
            _ => Ok(raw),
        }
    }
}

/// Offset of `m_first_child` within the final folder class, relative to the
/// node start. For a normal folder the base_folder_node IS the node start
/// (m_first_child @0). For the mount root, the embedded `folder` member sits at
/// offset 936 within the class, so m_first_child is at 936.
fn folder_start_within(flags: NodeFlags) -> usize {
    if flags.contains(NodeFlags::MOUNT_ROOT) {
        936 // archive_folder_mount_root_node::folder offset (m_first_child @ +0)
    } else {
        0
    }
}
