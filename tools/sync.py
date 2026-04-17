# -*- coding: utf-8 -*-
"""
Source Sync Tool v1
Sync source files from Trae workspace (D:) -> ADS workspace (E:)

Usage:
    python sync.py                  # Dry-run preview
    python sync.py --force          # Sync with overwrite
    python sync.py -f               # Short form
    python sync.py --verbose        # Show all files (including unchanged)
"""

import os
import sys
import shutil
import argparse
from datetime import datetime

SRC_DIR = r"D:\race\save\4.10\new_ins"
DST_DIR = r"E:\Aorcheved_01\new_ins"

SYNC_DIRS = ["code", "user", "libraries"]
SYNC_ROOT_FILES = [".cproject", ".project", "Lcf_Tasking_Tricore_Tc.lsl"]

EXTENSIONS = {".c", ".h"}


def get_file_size(path):
    try:
        return os.path.getsize(path)
    except OSError:
        return -1


def compare_files(src_path, dst_path):
    src_size = get_file_size(src_path)
    dst_size = get_file_size(dst_path)

    if dst_size < 0:
        return "NEW"
    if src_size != dst_size:
        return "DIFF"
    return "SAME"


def sync_one_file(src_path, dst_path, rel_name, dry_run, force, verbose, stats):
    result = compare_files(src_path, dst_path)

    if result == "NEW":
        if dry_run:
            print(f"  [NEW]     {rel_name}")
        else:
            os.makedirs(os.path.dirname(dst_path), exist_ok=True)
            shutil.copy2(src_path, dst_path)
            print(f"  [COPY]    {rel_name}")
        stats["copied"] += 1
        return

    if result == "DIFF":
        if force:
            if dry_run:
                s = get_file_size(src_path)
                d = get_file_size(dst_path)
                print(f"  [UPDATE]  {rel_name} ({s} vs {d} bytes)")
            else:
                shutil.copy2(src_path, dst_path)
                print(f"  [UPDATE]  {rel_name}")
            stats["updated"] += 1
        else:
            s = get_file_size(src_path)
            d = get_file_size(dst_path)
            print(f"  [DIFF]    {rel_name} ({s} vs {d} bytes) <- use -f")
            stats["skipped"] += 1
        return

    if verbose:
        print(f"  [OK]      {rel_name}")
    stats["skipped"] += 1


def sync_folder(dir_name, dry_run, force, verbose, stats):
    src_base = os.path.join(SRC_DIR, dir_name)
    dst_base = os.path.join(DST_DIR, dir_name)

    if not os.path.exists(src_base):
        print(f"  [SKIP]    {dir_name}\\ - not found")
        return

    count = 0
    for root, dirs, files in os.walk(src_base):
        dirs.sort()
        for fname in sorted(files):
            ext = os.path.splitext(fname)[1].lower()
            if ext not in EXTENSIONS:
                continue

            src_full = os.path.join(root, fname)
            rel = os.path.relpath(src_full, src_base)
            dst_full = os.path.join(dst_base, rel)

            count += 1
            rel_display = f"{dir_name}\\{rel}"
            sync_one_file(src_full, dst_full, rel_display, dry_run, force, verbose, stats)

    if count == 0:
        print(f"  [EMPTY]   {dir_name}\\")


def main():
    parser = argparse.ArgumentParser(
        description="Sync D:\\race workspace -> E:\\ADS workspace",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("-f", "--force", action="store_true", help="Overwrite differing files")
    parser.add_argument("-n", "--dry-run", action="store_true", help="Preview only (default)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Show unchanged files too")
    args = parser.parse_args()

    dry_run = True
    if args.force:
        dry_run = False

    print("=" * 50)
    print("  Source Sync: Trae -> ADS")
    print("=" * 50)
    print(f"  Source : {SRC_DIR}")
    print(f"  Target : {DST_DIR}")
    print(f"  Mode   : {'DRY RUN' if dry_run else 'SYNC' + (' + FORCE' if args.force else '')}")
    print("=" * 50)
    print()

    if not os.path.exists(SRC_DIR):
        print(f"[ERROR] Source not found: {SRC_DIR}")
        sys.exit(1)
    if not os.path.exists(DST_DIR):
        print(f"[ERROR] Target not found: {DST_DIR}")
        sys.exit(1)

    stats = {"copied": 0, "updated": 0, "skipped": 0}

    print("[INFO] Root config files:")
    for fname in SYNC_ROOT_FILES:
        src = os.path.join(SRC_DIR, fname)
        dst = os.path.join(DST_DIR, fname)
        if os.path.exists(src):
            sync_one_file(src, dst, fname, dry_run, args.force, args.verbose, stats)

    print()
    for dir_name in SYNC_DIRS:
        print(f"[INFO] Scanning {dir_name}\\")
        sync_folder(dir_name, dry_run, args.force, args.verbose, stats)
        print()

    print("=" * 50)
    print(f"  Copied  : {stats['copied']}")
    print(f"  Updated : {stats['updated']}")
    print(f"  Skipped : {stats['skipped']}")
    print("=" * 50)

    if dry_run:
        print("\n[INFO] Dry run complete - no changes made")
        print("To actually sync, run: python sync.py --force")
    elif stats["updated"] > 0:
        print("\n[DONE] Synced! Next: Build in ADS IDE (Ctrl+B)")
    else:
        print("\n[INFO] All up to date")


if __name__ == "__main__":
    main()
