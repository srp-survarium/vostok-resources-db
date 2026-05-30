# Node layouts & the "negative offsets"

Every stored node pointer in the FAT (`m_next`, `m_parent`, `m_first_child`,
`referenced`, the mount-root's `node`, …) points **at a node's embedded
`base_node`** — never at the start of the node's concrete class.

But the class-specific fields (`archive_file_node_base`, the inline data, a
folder's `m_first_child`, a link's `referenced`, …) sit **before** `base_node`
in memory. So to read them we go **backwards** from the pointer:

```
class start = base_node - base_off(kind)
```

`base_off` is the offset of `base_node` *within* that concrete class. This file
draws every class so you can see why each subtraction lands where it does.

Legend: `[ field (size) ]`, offsets are byte offsets from the **class start**
(offset 0). `^base` marks where a stored pointer points; `^start` marks the
class start you recover by subtracting `base_off`.

---

## `base_node` itself (the common header)

```
 0    8    16   24   32   40   48  50 51
 +----+----+----+----+----+----+---+--+--------
 |un- |nxt |hsh |NEXT|par |asc |FLG|lk| name[] ...
 |ion |ovl |set |    |ent |oc  |u16|u8| (NUL-term, flexible)
 +----+----+----+----+----+----+---+--+--------
 (each ptr field is 8 bytes; sizeof = 56, i.e. 51 rounded up to 8)
```

We only read `m_next` (@24), `m_flags` (@48) and `m_name` (@51), all relative to
the `base_node` pointer itself (these are **positive** offsets into base_node).
The negative offsets below are for reaching the *enclosing class*.

---

## File nodes

### `archive_file_node`  —  `base_off = 24`

```
 0         8           16    20    24
 +---------+-----------+-----+-----+------------------------ ... ----
 | size_in | pos_in_db | hash| pad | base_node ( flags@+48, name@+51 )
 | _db u32 |    u64    | u32 |     |
 +---------+-----------+-----+-----+------------------------ ... ----
 ^start                            ^base   (pointer target)
 |<-------- base_off = 24 -------->|

 read:  afnb = read(base - 24)           // size_in_db@0, pos_in_db@8, hash@16
```

### `archive_compressed_file_node`  —  `base_off = 32`

```
 0         8           16    20    24       28    32
 +---------+-----------+-----+-----+--------+-----+------------ ... ----
 | size_in | pos_in_db | hash| pad | u32    | pad | base_node
 | _db     |           |     |     | ucsize |     |
 +---------+-----------+-----+-----+--------+-----+------------ ... ----
 ^start                            ^uncomp        ^base
 |<-------------- base_off = 32 ----------------->|

 read:  afnb            = read(base - 32)
        uncompressed_sz = read(base - 8)          // = node @24
```

### `archive_inline_file_node`  —  `base_off = 40`

The small file's bytes live **inside the FAT buffer**; `m_inlined_data` is a
buffer offset to them.

```
 0         8     16    24                36    40
 +---------+-----+-----+-----------------+-----+------------ ... ----
 |  afnb (size_in_db / pos / hash)       | base_node
 |  (24 bytes)     | m_inlined_data u64  |
 |                 | m_inlined_size  u32 |
 +---------+-----+-----+-----------------+-----+------------ ... ----
 ^start              ^aifnb (@24)              ^base
 |<------------- base_off = 40 ------------------>|

 read:  afnb           = read(base - 40)
        m_inlined_data = read(base - 40 + AIFNB_OFF(24))   // = node @24
```

### `archive_inline_compressed_file_node`  —  `base_off = 48`

```
 0           24                40      44   48
 +-----------+-----------------+-------+----+---------- ... ----
 | afnb (24) | aifnb (16)      | ucs   | pad| base_node
 +-----------+-----------------+-------+----+---------- ... ----
 ^start                                     ^base
 |<--------------- base_off = 48 ---------->|
```

---

## Folders

For folders the stored child pointer points at the folder's `base_node`, and
`m_first_child` is **16 bytes before it** (`base - 16`) in *both* the normal and
mount-root cases.

### `base_folder_node`  —  `base_off = 16`, `folder_start_within = 0`

```
 0              8          16
 +--------------+----------+----------------- ... ----
 | m_first_child| counters | base_node
 |     u64      |          |
 +--------------+----------+----------------- ... ----
 ^start/first_child        ^base
 |<--- base_off = 16 ----->|

 read:  first_child = read(base - 16 + 0)
```

### `archive_folder_mount_root_node`  —  `base_off = 952`, `folder_start_within = 936`

The folder is embedded deep inside the mount-root class, but the two offsets
cancel so `m_first_child` is again at `base - 16`:

```
 0                                            936          952
 +--------------- mount_root_node_base etc ---+------------+--------- ... ----
 | (104) + 3*char[260] + char[32] + 2*ptr     | folder:    | base_node
 |   ...  node ptr @64 (-> root base_node) ... | first_child| (of folder)
 |                                            | @936       | @952
 +--------------------------------------------+------------+--------- ... ----
 ^start                                       ^first_child  ^base

 read:  first_child = read(base - base_off(952) + folder_start_within(936))
                    = read(base - 16)
```

The `node` pointer at **buffer offset 64** of the mount-root holds the buffer
offset of the *root folder's* `base_node` — that's where the whole walk starts.

---

## Links, erased, external

### `hard_link_node` / `soft_link_node`  —  `base_off = 8`

```
 0            8
 +------------+----------------- ... ----
 | referenced | base_node
 |    u64     |
 +------------+----------------- ... ----
 ^start       ^base
 |<- off=8 -->|

 read:  referenced = read(base - 8)     // -> the referenced node's base_node
```

For a **hard link** we then resolve `referenced` (it points at another node's
`base_node`) and read *its* file fields, keeping this node's own path.

### `erased_node`  —  `base_off = 0`

```
 0
 +----------------- ... ----
 | base_node                 (base_node IS the class start; nothing precedes it)
 +----------------- ... ----
 ^start == ^base
```

### `external_subfat_node`  —  `base_off = 16`

```
 0                 16
 +-----------------+----------------- ... ----
 | 16-byte prefix  | base_node
 +-----------------+----------------- ... ----
 ^start            ^base
 |<-- off = 16 --->|
```

---

## Summary of every negative read

| node kind                | `base_off` | reads at `base - …`                          |
|--------------------------|-----------:|----------------------------------------------|
| file                     | 24         | afnb @ −24                                    |
| compressed file          | 32         | afnb @ −32, uncompressed_size @ −8            |
| inline file              | 40         | afnb @ −40, m_inlined_data @ −40+24 = −16     |
| inline compressed file   | 48         | afnb @ −48, …                                 |
| folder                   | 16         | m_first_child @ −16                           |
| mount-root folder        | 952        | m_first_child @ −952+936 = −16                |
| hard / soft link         | 8          | referenced @ −8                               |
| erased                   | 0          | (nothing before base_node)                    |
| external sub-fat         | 16         | (base_node @ −16; prefix not read)            |
