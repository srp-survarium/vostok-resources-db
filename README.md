# resources-db

Unpacker for the Survarium / Vostok engine `resources.db` VFS pack archive.

## Build & run

```sh
nix shell nixpkgs#cargo nixpkgs#rustc nixpkgs#gcc --command cargo build --release

# extract everything, reconstructing the folder tree
./target/release/resources-db <resources.db> <output-dir>

# just list the contents (kind, size_in_db, uncompressed_size, pos_in_db, path)
./target/release/resources-db --list <resources.db>
```

## On-disk format

The format is derived from the engine source under `sources/vostok/vfs/`.

```
[ fat_header (24 bytes) ][ FAT node buffer (buffer_size bytes) ][ data blob ]
```

`fat_header` (`vfs/sources/fat_header.h`): `char endian_string[14]` ("little-endian\0"),
2 pad bytes, `u32 num_nodes`, `u32 buffer_size`.

The FAT node buffer is a packed tree of variable-length nodes. Node "pointers"
(`m_next`, `m_parent`, `m_first_child`, `referenced`, ...) are byte **offsets
into the FAT node buffer** (offset 0 == NULL). The archive is built with the PC
archive layout, which uses **64-bit** pointers and a **u64** `file_size_type`
(`saving_db.cpp` selects `platform_pointer_64bit` for `archive_platform_pc`).

A stored node pointer points at the node's embedded `base_node`. The start of
the final node class is `base_node_offset - base_off(kind)`, where `base_off`
depends on the node flags. File payload fields (`archive_file_node_base`:
`u32 size_in_db; u64 pos_in_db; u32 hash`) live at the start of the final class.

`pos_in_db` is the **absolute file offset** of the payload. Compressed files
add a `u32 uncompressed_size` immediately before `base_node`. Inline files store
their bytes inside the FAT buffer (`m_inlined_data` is a buffer offset).
Hard-link nodes (`is_hard_link`) reference another node's payload and are
resolved during extraction.

### Verified 64-bit struct offsets (MSVC `#pragma pack(8)`)

```
base_node:          m_next @24  m_parent @32  m_flags @48  m_name @51
archive_file_node_base: size_in_db @0  pos_in_db @8  hash @16
base_off (offset of base_node within the final class):
  folder 16, file 24, compressed 32, inline 40, inline_compressed 48,
  soft_link 8, hard_link 8, erased 0, external 16, mount_root_folder 952
folder: m_first_child @0 (relative to base_folder_node start)
```

The root folder's `base_node` offset is read from the mount-root node's `node`
pointer field at buffer offset 64.

## Compression: PPMd

Compressed payloads use a **custom Dmitry Shkarin PPMd (PPMII)** variant, ported
in `src/ppmd.rs` from `sources/vostok/core/sources/compressor_ppmd*.{cpp,h}`:

* model order **8** (`order_model`), `model_restoration_restart`, 32 MiB
  sub-allocator (`ppmd_compressor(allocator, 32)` in `pack_archive.cpp`);
* Subbotin carryless range coder, `TOP = 1<<24`, **`BOT = 1<<15`** (note: 1<<15,
  not the usual 1<<16), 4-byte decoder init;
* Win32 build => `UNIT_SIZE = 12`, 4-byte in-heap pointers. The sub-allocator's
  32-bit address space is emulated with a byte buffer plus a virtual base.

This is NOT 7-zip's PPMd7/PPMd8 and cannot be replaced by an off-the-shelf
crate. The shipped `resources.db` happens to contain **no** compressed or inline
files (everything is stored raw), so the PPMd decoder is ported but not yet
verified end-to-end against real compressed data.
