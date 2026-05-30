use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use resources_db::{Archive, NodeKind};

fn usage() -> ! {
    eprintln!("usage:");
    eprintln!("  resources-db <resources.db> <output-dir>   extract all files");
    eprintln!("  resources-db --list <resources.db>         list contents");
    std::process::exit(2);
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();

    if args.len() == 3 && args[1] == "--list" {
        return match run_list(&args[2]) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        };
    }

    if args.len() != 3 {
        usage();
    }

    match run_extract(&args[1], &args[2]) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run_list(db: &str) -> Result<(), Box<dyn std::error::Error>> {
    let archive = Archive::open(db)?;
    eprintln!("num_nodes={} buffer_size={}", archive.num_nodes, archive.buffer_size);
    let entries = archive.list();
    let mut files = 0usize;
    let mut compressed = 0usize;
    let mut inline = 0usize;
    for e in &entries {
        let tag = match e.kind {
            NodeKind::File { compressed: c, inlined: i } => {
                files += 1;
                if c {
                    compressed += 1;
                }
                if i {
                    inline += 1;
                }
                match (c, i) {
                    (true, true) => "file/ppmd/inline",
                    (true, false) => "file/ppmd",
                    (false, true) => "file/inline",
                    (false, false) => "file",
                }
            }
            NodeKind::SoftLink => "soft-link",
            NodeKind::HardLink => "hard-link",
            NodeKind::Erased => "erased",
            NodeKind::External => "external",
            NodeKind::Folder => "folder",
            NodeKind::Other => "other",
        };
        println!(
            "{:<18} {:>12} {:>12} {:>12}  {}",
            tag, e.size_in_db, e.uncompressed_size, e.pos_in_db, e.path
        );
    }
    eprintln!(
        "files={} compressed={} inline={} total_entries={}",
        files,
        compressed,
        inline,
        entries.len()
    );
    Ok(())
}

fn run_extract(db: &str, out_dir: &str) -> Result<(), Box<dyn std::error::Error>> {
    let mut archive = Archive::open(db)?;
    eprintln!("num_nodes={} buffer_size={}", archive.num_nodes, archive.buffer_size);
    let entries = archive.list();
    let out_root = PathBuf::from(out_dir);

    let mut extracted = 0usize;
    let mut skipped = 0usize;
    for e in &entries {
        match e.kind {
            NodeKind::File { .. } => {}
            _ => {
                skipped += 1;
                continue;
            }
        }
        let rel = sanitize(&e.path);
        let dest = out_root.join(&rel);
        if let Some(parent) = dest.parent() {
            fs::create_dir_all(parent)?;
        }
        let data = archive.read_file(e)?;
        let mut f = fs::File::create(&dest)
            .map_err(|err| format!("creating {}: {err}", dest.display()))?;
        f.write_all(&data)?;
        extracted += 1;
        if extracted % 1000 == 0 {
            eprintln!("  {extracted}/{} ...", entries.len());
        }
    }
    eprintln!("extracted {extracted} files ({skipped} non-file entries skipped) into {out_dir}");
    Ok(())
}

/// Strip any path components that could escape the output dir.
fn sanitize(path: &str) -> PathBuf {
    let mut out = PathBuf::new();
    for comp in path.split(['/', '\\']) {
        match comp {
            "" | "." => {}
            ".." => {}
            other => out.push(Path::new(other)),
        }
    }
    out
}
