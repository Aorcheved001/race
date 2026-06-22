#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ADS Workspace Sync Script
Syncs code/ and user/ folders to ADS workspace with GB2312 encoding conversion.
Run after each code modification.
"""

import os
import sys
import shutil
from pathlib import Path

# === Configuration ===
SRC_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Seekfree_TC377_Opensource_Library")
DST_ROOT = r"E:\Aorcheved_01\Subject_03"
SYNC_DIRS = ["code", "user"]
ENCODING_SRC = "utf-8"
ENCODING_DST = "gb2312"
SKIP_EXTENSIONS = {".o", ".d", ".src", ".elf", ".hex", ".map", ".bat"}
SKIP_DIRS = {"__pycache__", ".git"}


def convert_file(src_path, dst_path):
    """Convert file encoding from UTF-8 to GB2312"""
    try:
        with open(src_path, 'r', encoding=ENCODING_SRC) as f:
            content = f.read()
    except UnicodeDecodeError:
        # If not UTF-8, try reading as-is and write directly
        shutil.copy2(src_path, dst_path)
        return True
    
    try:
        with open(dst_path, 'w', encoding=ENCODING_DST) as f:
            f.write(content)
    except UnicodeEncodeError:
        # Fall back to GBK if GB2312 can't encode some chars
        try:
            with open(dst_path, 'w', encoding='gbk') as f:
                f.write(content)
        except Exception as e:
            print(f"  [WARN] Encoding failed for {src_path}: {e}")
            shutil.copy2(src_path, dst_path)
    return True


def sync_dir(src_dir, dst_dir, dry_run=False, force=False):
    """Sync one directory tree"""
    synced = 0
    skipped = 0
    errors = 0
    
    for root, dirs, files in os.walk(src_dir):
        # Skip hidden/unnecessary dirs
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        
        rel_root = os.path.relpath(root, src_dir)
        dst_root = os.path.join(dst_dir, rel_root)
        
        for fname in files:
            ext = os.path.splitext(fname)[1].lower()
            if ext in SKIP_EXTENSIONS:
                continue
            
            src_file = os.path.join(root, fname)
            dst_file = os.path.join(dst_root, fname)
            
            # Check if sync needed (compare modification time)
            if not force and os.path.exists(dst_file):
                src_mtime = os.path.getmtime(src_file)
                dst_mtime = os.path.getmtime(dst_file)
                if src_mtime <= dst_mtime:
                    skipped += 1
                    continue
            
            if dry_run:
                print(f"  [DRY] {os.path.relpath(src_file, SRC_ROOT)} -> {os.path.relpath(dst_file, DST_ROOT)}")
                synced += 1
                continue
            
            # Create destination directory
            os.makedirs(dst_root, exist_ok=True)
            
            # Convert text files, copy binary files
            if ext in {'.c', '.h', '.md', '.txt', '.lsl', '.cfg'}:
                try:
                    convert_file(src_file, dst_file)
                    synced += 1
                except Exception as e:
                    print(f"  [ERR] {fname}: {e}")
                    errors += 1
            else:
                shutil.copy2(src_file, dst_file)
                synced += 1
    
    return synced, skipped, errors


def main():
    dry_run = '--dry-run' in sys.argv or '-n' in sys.argv
    force = '--force' in sys.argv or '-f' in sys.argv
    
    if not os.path.exists(SRC_ROOT):
        print(f"[ERROR] Source not found: {SRC_ROOT}")
        return 1
    
    if not os.path.exists(DST_ROOT):
        print(f"[ERROR] Destination not found: {DST_ROOT}")
        return 1
    
    mode = "DRY RUN" if dry_run else "SYNC"
    print(f"[{mode}] {SRC_ROOT}")
    print(f"    -> {DST_ROOT}")
    print(f"    Encoding: {ENCODING_SRC} -> {ENCODING_DST}")
    print()
    
    total_synced = 0
    total_skipped = 0
    total_errors = 0
    
    for d in SYNC_DIRS:
        src_dir = os.path.join(SRC_ROOT, d)
        dst_dir = os.path.join(DST_ROOT, d)
        
        if not os.path.exists(src_dir):
            print(f"[WARN] Source dir missing: {src_dir}")
            continue
        
        print(f"Syncing {d}/ ...")
        synced, skipped, errors = sync_dir(src_dir, dst_dir, dry_run, force)
        print(f"  Synced: {synced}, Skipped(up-to-date): {skipped}, Errors: {errors}")
        total_synced += synced
        total_skipped += skipped
        total_errors += errors
    
    print()
    print(f"Done. Total synced={total_synced}, skipped={total_skipped}, errors={total_errors}")
    return 1 if total_errors > 0 else 0


if __name__ == "__main__":
    sys.exit(main())
