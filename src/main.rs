use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{Parser, Subcommand};
use resources_db::{Archive, NodeKind};

/// Unpacker for the Survarium / Vostok engine `resources.db` VFS pack archive.
#[derive(Parser)]
#[command(version, about)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// List the archive's contents.
    List {
        /// Path to resources.db.
        db: PathBuf,
    },
    /// Extract all files into a directory.
    Extract {
        /// Path to resources.db.
        db: PathBuf,
        /// Output directory.
        out_dir: PathBuf,
    },
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let result = match cli.command {
        Command::List { db } => run_list(&db),
        Command::Extract { db, out_dir } => run_extract(&db, &out_dir),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run_list(db: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let archive = Archive::open(db)?;
    eprintln!(
        "num_nodes={} buffer_size={}",
        archive.num_nodes, archive.buffer_size
    );
    let entries = archive.list();
    let (mut files, mut compressed, mut inline) = (0usize, 0usize, 0usize);
    for e in &entries {
        let tag = match e.kind {
            NodeKind::File {
                compressed: c,
                inlined: i,
            } => {
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

fn run_extract(db: &Path, out_dir: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let mut archive = Archive::open(db)?;
    eprintln!(
        "num_nodes={} buffer_size={}",
        archive.num_nodes, archive.buffer_size
    );
    let entries = archive.list();

    let (mut extracted, mut skipped) = (0usize, 0usize);
    for e in &entries {
        if !matches!(e.kind, NodeKind::File { .. }) {
            skipped += 1;
            continue;
        }
        let dest = out_dir.join(sanitize(&e.path));
        if let Some(parent) = dest.parent() {
            fs::create_dir_all(parent)?;
        }
        let data = archive.read_file(e)?;
        let mut f =
            fs::File::create(&dest).map_err(|err| format!("creating {}: {err}", dest.display()))?;
        f.write_all(&data)?;
        extracted += 1;
        if extracted % 1000 == 0 {
            eprintln!("  {extracted}/{} ...", entries.len());
        }
    }
    eprintln!(
        "extracted {extracted} files ({skipped} non-file entries skipped) into {}",
        out_dir.display()
    );
    Ok(())
}

/// Strip any path components that could escape the output dir.
fn sanitize(path: &str) -> PathBuf {
    let mut out = PathBuf::new();
    for comp in path.split(['/', '\\']) {
        match comp {
            "" | "." | ".." => {}
            other => out.push(Path::new(other)),
        }
    }
    out
}
