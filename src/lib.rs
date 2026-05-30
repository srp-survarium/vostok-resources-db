//! Reader/unpacker for the Survarium / Vostok engine `resources.db` VFS pack
//! archive.
//!
//! The on-disk format (little-endian, built for the 64-bit PC archive layout —
//! `archive_platform_pc` uses `platform_pointer_64bit` in `saving_db.cpp`):
//!
//! ```text
//! [ fat_header (24 bytes) ][ FAT node buffer (buffer_size bytes) ][ data blob ]
//! ```
//!
//! The FAT node buffer is a packed tree of variable-length nodes. Node
//! "pointers" are 64-bit byte offsets relative to the start of the FAT node
//! buffer (offset 0 == NULL). File payloads live in the data blob and are
//! located by absolute file offset (`pos_in_db`).
//!
//! PPMd-compressed payloads use a custom Dmitry Shkarin PPMd variant (order 8,
//! Subbotin range coder with BOT=1<<15); see [`ppmd`].

pub mod fat;
// Several allocator/model helpers in the PPMd port are only exercised by the
// cut_off / freeze model-restoration paths (this build uses
// model_restoration_restart). They are kept faithful to the C++ source.
#[allow(dead_code)]
pub mod ppmd;

pub use fat::{Archive, NodeKind, FileEntry};
