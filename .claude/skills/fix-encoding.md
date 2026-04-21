# fix-encoding

Fix Chinese comment encoding issues in C/H source files, convert to GB2312 and sync to ADS workspace.

## Usage

```
/fix-encoding <file_path>
```

Or batch fix:
```
/fix-encoding <file1> <file2> ...
```

## Workflow

### Step 1: Use Write tool to write fixed content (UTF-8 Chinese)

```
Write file_path="d:/race/save/4.10 - vs/new_ins/code/control/example.c"
content="/* correct Chinese comments */ ..."
```

### Step 2: Use PowerShell to convert to GB2312 encoding

```powershell
powershell -Command "Get-Content -Path '<file>' -Encoding UTF8 | Set-Content -Path '<file_temp>' -Encoding Default; Move-Item -Path '<file_temp>' -Destination '<file>' -Force"
```

### Step 3: Sync to ADS workspace

```bash
cp -v "<new_ins_path>" "E:/Aorcheved_01/new_ins/<relative_path>"
```

## Path Information

- Source path: `d:/race/save/4.10 - vs/new_ins/`
- ADS workspace: `E:/Aorcheved_01/new_ins/`
- Reference (brother computer): `d:/race/save/4.10 - vs/兄弟计算机/`

## Important Notes

- C/H source files MUST use GB2312 encoding
- md/json/xml files MUST use UTF-8 encoding
- DO NOT use Python scripts for encoding, it will cause corruption
- If brother computer has correct version, copy directly

## Quick Fix Command

If brother computer has correct version:
```bash
cp -v "d:/race/save/4.10 - vs/兄弟计算机/code/control/<file>" "d:/race/save/4.10 - vs/new_ins/code/control/<file>"
cp -v "d:/race/save/4.10 - vs/new_ins/code/control/<file>" "E:/Aorcheved_01/new_ins/code/control/<file>"
```
