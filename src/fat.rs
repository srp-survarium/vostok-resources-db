//! Parsing of the FAT header and the packed node tree.

use std::fs::File;
use std::io::{self, Read, Seek, SeekFrom};
use std::path::Path;

// ---- fat_header (sources/vostok/vfs/sources/fat_header.h) -------------------
// struct fat_header { char endian_string[14]; /*2 pad*/ u32 num_nodes; u32 buffer_size; };
pub const HEADER_SIZE: u64 = 24;

// ---- vfs_node_enum flags (sources/vostok/vfs/base_node.h) -------------------
pub mod flags {
    pub const IS_FOLDER: u16 = 1 << 0;
    pub const IS_PHYSICAL: u16 = 1 << 1;
    pub const IS_ARCHIVE: u16 = 1 << 2;
    pub const IS_MOUNT_ROOT: u16 = 1 << 3;
    pub const IS_COMPRESSED: u16 = 1 << 4;
    pub const IS_REPLICATED: u16 = 1 << 5;
    pub const IS_INLINED: u16 = 1 << 6;
    pub const IS_SUB_FAT: u16 = 1 << 7;
    pub const IS_SOFT_LINK: u16 = 1 << 8;
    pub const IS_HARD_LINK: u16 = 1 << 9;
    pub const IS_MOUNT_HELPER: u16 = 1 << 10;
    pub const IS_ERASED: u16 = 1 << 11;
    pub const IS_EXTERNAL_SUB_FAT: u16 = 1 << 12;
    pub const IS_UNIVERSAL_FILE: u16 = 1 << 13;
    pub const IS_MARKED_TO_UNLINK: u16 = 1 << 14;
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

// archive_file_node_base: u32 size_in_db @0; u64 pos_in_db @8; u32 hash @16.
// archive_inline_file_node_base: ptr m_inlined_data @0; u32 m_inlined_size @8
// (relative to the start of the aifnb sub-object, which immediately follows
//  the 24-byte afnb sub-object).
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
        let mut header = [0u8; HEADER_SIZE as usize];
        file.read_exact(&mut header)?;

        let endian = &header[0..13];
        if endian != b"little-endian" {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("unexpected endian string: {:?}", String::from_utf8_lossy(&header[0..14])),
            ));
        }
        let num_nodes = u32::from_le_bytes(header[16..20].try_into().unwrap());
        let buffer_size = u32::from_le_bytes(header[20..24].try_into().unwrap());

        let mut buf = vec![0u8; buffer_size as usize];
        file.read_exact(&mut buf)?;

        Ok(Archive { file, buf, num_nodes, buffer_size })
    }

    fn u16_at(&self, off: usize) -> u16 {
        u16::from_le_bytes(self.buf[off..off + 2].try_into().unwrap())
    }
    fn u32_at(&self, off: usize) -> u32 {
        u32::from_le_bytes(self.buf[off..off + 4].try_into().unwrap())
    }
    fn u64_at(&self, off: usize) -> u64 {
        u64::from_le_bytes(self.buf[off..off + 8].try_into().unwrap())
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
        let ptr = self.u64_at(MOUNT_ROOT_NODE_PTR_OFF) as usize;
        if ptr != 0 && ptr < self.buf.len() {
            ptr
        } else {
            BASE_OFF_MOUNT_ROOT
        }
    }

    fn kind_for(&self, flags: u16) -> NodeKind {
        if flags & flags::IS_FOLDER != 0 {
            NodeKind::Folder
        } else if flags & flags::IS_SOFT_LINK != 0 {
            NodeKind::SoftLink
        } else if flags & flags::IS_HARD_LINK != 0 {
            NodeKind::HardLink
        } else if flags & flags::IS_ERASED != 0 {
            NodeKind::Erased
        } else if flags & flags::IS_EXTERNAL_SUB_FAT != 0 {
            NodeKind::External
        } else if flags & flags::IS_ARCHIVE != 0 {
            NodeKind::File {
                compressed: flags & flags::IS_COMPRESSED != 0,
                inlined: flags & flags::IS_INLINED != 0,
            }
        } else {
            NodeKind::Other
        }
    }

    /// base_off (offset of base_node within the final node class) for `flags`.
    fn base_off_for(&self, flags: u16) -> usize {
        if flags & flags::IS_MOUNT_ROOT != 0 {
            BASE_OFF_MOUNT_ROOT
        } else if flags & flags::IS_FOLDER != 0 {
            BASE_OFF_FOLDER
        } else if flags & flags::IS_SOFT_LINK != 0 {
            BASE_OFF_SOFT_LINK
        } else if flags & flags::IS_HARD_LINK != 0 {
            BASE_OFF_HARD_LINK
        } else if flags & flags::IS_ERASED != 0 {
            BASE_OFF_ERASED
        } else if flags & flags::IS_EXTERNAL_SUB_FAT != 0 {
            BASE_OFF_EXTERNAL
        } else if flags & flags::IS_INLINED != 0 {
            if flags & flags::IS_COMPRESSED != 0 {
                BASE_OFF_INLINE_COMPRESSED
            } else {
                BASE_OFF_INLINE
            }
        } else if flags & flags::IS_COMPRESSED != 0 {
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
        let root_flags = self.u16_at(root + BASE_NODE_FLAGS);
        debug_assert!(root_flags & flags::IS_FOLDER != 0);
        self.walk_folder_children(root, "", &mut out);
        out
    }

    fn first_child_of_folder(&self, base_node_off: usize, flags: u16) -> usize {
        // base_folder_node: m_first_child is at (base_node - 16) for normal
        // folders, and at (folder.base - 16) for the mount root too (folder
        // is a base_folder_node embedded in the mount root).
        let folder_start = base_node_off - self.base_off_for(flags) + folder_start_within(flags);
        self.u64_at(folder_start) as usize
    }

    fn walk_folder_children(&self, folder_base_node_off: usize, prefix: &str, out: &mut Vec<FileEntry>) {
        let flags = self.u16_at(folder_base_node_off + BASE_NODE_FLAGS);
        let mut child = self.first_child_of_folder(folder_base_node_off, flags);
        while child != 0 {
            self.visit_node(child, prefix, out);
            child = self.u64_at(child + BASE_NODE_NEXT) as usize;
        }
    }

    fn visit_node(&self, base_node_off: usize, prefix: &str, out: &mut Vec<FileEntry>) {
        let flags = self.u16_at(base_node_off + BASE_NODE_FLAGS);
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
                let ref_base_off = self.u64_at(node_start) as usize;
                if ref_base_off != 0 && ref_base_off < self.buf.len() {
                    let ref_flags = self.u16_at(ref_base_off + BASE_NODE_FLAGS);
                    let mut entry = self.read_file_fields(ref_base_off, ref_flags, path);
                    // Mark its kind as resolved-from-hard-link by keeping the
                    // referenced file kind (so extraction/decompression works).
                    entry.kind = self.kind_for(ref_flags);
                    out.push(entry);
                } else {
                    out.push(FileEntry { path, kind, size_in_db: 0, uncompressed_size: 0, pos_in_db: 0, inline_buffer_offset: None });
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
    fn read_file_fields(&self, base_node_off: usize, flags: u16, path: String) -> FileEntry {
        let compressed = flags & flags::IS_COMPRESSED != 0;
        let inlined = flags & flags::IS_INLINED != 0;
        let node_start = base_node_off - self.base_off_for(flags);
        let size_in_db = self.u32_at(node_start); // afnb.size_in_db
        let pos_in_db = self.u64_at(node_start + 8); // afnb.pos_in_db
        let uncompressed_size = if compressed {
            // archive(_inline)_compressed_file_node: u32 uncompressed_size,
            // located right before `base`.
            self.u32_at(base_node_off - 8)
        } else {
            size_in_db
        };
        let inline_buffer_offset = if inlined {
            // aifnb.m_inlined_data holds the buffer offset of the data.
            Some(self.u64_at(node_start + AIFNB_OFF))
        } else {
            None
        };
        FileEntry {
            path,
            kind: NodeKind::File { compressed, inlined },
            size_in_db,
            uncompressed_size,
            pos_in_db,
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

    /// Read and (if needed) decompress the payload for a file entry.
    pub fn read_file(&mut self, e: &FileEntry) -> io::Result<Vec<u8>> {
        let raw = self.read_raw(e)?;
        match e.kind {
            NodeKind::File { compressed: true, .. } => {
                let mut out = vec![0u8; e.uncompressed_size as usize];
                crate::ppmd::decompress(&raw, &mut out).map_err(|err| {
                    io::Error::new(io::ErrorKind::InvalidData, format!("ppmd: {err}"))
                })?;
                Ok(out)
            }
            _ => Ok(raw),
        }
    }
}

/// Offset of `m_first_child` within the final folder class, relative to the
/// node start. For a normal folder the base_folder_node IS the node start
/// (m_first_child @0). For the mount root, the embedded `folder` member sits at
/// offset 936 within the class, so m_first_child is at 936.
fn folder_start_within(flags: u16) -> usize {
    if flags & flags::IS_MOUNT_ROOT != 0 {
        936 // archive_folder_mount_root_node::folder offset (m_first_child @ +0)
    } else {
        0
    }
}
