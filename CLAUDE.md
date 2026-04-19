# AURIX TC377 new_ins ÏîÄ¿ÅäÖÃ

> ÏîÄ¿: new_ins (AURIX TC377 ¹ßÐÔµ¼º½ÏµÍ³)
> ADS °æ±¾: v1.10.2
> Ð¾Æ¬: TC377TP

---

## ÏîÄ¿¸ÅÊö

±¾ÏîÄ¿ÊÇ»ùÓÚ Infineon AURIX TC377 µÄ¹ßÐÔµ¼º½ÏµÍ³ (INS)£¬°üº¬£º
- EKF ÈÚºÏ×ËÌ¬¹À¼Æ
- µç»ú/±àÂëÆ÷¿ØÖÆ
- GNSS/GPS ½ÓÊÕ
- ÎÞÏßÊý¾Ý´«Êä

---

## ÖØÒªÔ¼Êø

### ±àÂëÔ¼Êø
**C/C++ Ô´ÎÄ¼þÊ¹ÓÃ GB2312 ±àÂë£¡**
- `.c` / `.h` ÎÄ¼þÊ¹ÓÃ GB2312
- `.md` / `.json` / `.xml` µÈ±£³Ö UTF-8

### ¹¹½¨Ô¼Êø
**±ØÐëÊ¹ÓÃ ADS IDE ¹¹½¨£¡**
- ÐÞ¸Ä´úÂëºó£¬ÔÚ ADS IDE ÖÐ°´ Ctrl+B ¹¹½¨
- ²»ÄÜÊ¹ÓÃÃüÁîÐÐ±àÒë (Tasking ·ÇÉÌÒµ°æÏÞÖÆ)
- ¹¹½¨³É¹¦ºó£¬¿ÉÓÃ `tools\headless_build.bat --flash-only` ÉÕÂ¼

---

## ±Ø¶ÁÎÄµµ

1. **docs/TOOLCHAIN_GUIDE.md** - ¹¤¾ßÁ´Ô¼ÊøÖ¸ÄÏ
2. **docs/PROJECT_INDEX.md** - ÏîÄ¿Ë÷Òý
3. **docs/INS_IMPLEMENTATION_SUMMARY.md** - INS ÊµÏÖÏ¸½Ú

---

## ¹Ø¼üÂ·¾¶

```
code/
©À©¤©¤ system/init_all.c    # ËùÓÐÍâÉè³õÊ¼»¯
©À©¤©¤ system/interrupt.c   # ÖÐ¶ÏÈÎÎñµ÷¶È
©À©¤©¤ ins/Ins.c           # INS ºËÐÄÒýÇæ
©¸©¤©¤ control/            # µç»ú/PID/±àÂëÆ÷

tools/headless_build.bat  # ¹¹½¨+ÉÕÂ¼½Å±¾
scripts/serial_reader.py   # ´®¿ÚÊý¾Ý²É¼¯
```

---

## ³£ÓÃ²Ù×÷

| ²Ù×÷ | ÃüÁî |
|------|------|
| ÉÕÂ¼¹Ì¼þ | `tools\headless_build.bat --flash-only` |
| ²É¼¯´®¿ÚÊý¾Ý | `python scripts\serial_reader.py -t 60 --save` |

---

## µ÷ÊÔ¶Ë¿Ú

- **COM33**: Debug Port (Infineon DAS JDS) - ÉÕÂ¼ºÍµ÷ÊÔ
- **COM24**: Wireless Data Port (USB-SERIAL, 115200bps) - Êý¾ÝÊä³ö

---

## Ë«Â·¾¶¼Ü¹¹

| Â·¾¶ | ÓÃÍ¾ |
|------|------|
| `D:\race\save\4.10 - vs\new_ins` | VS Code ¹¤×÷Çø - ´úÂë±à¼­ |
| `E:\Aorcheved_01\new_ins` | ADS ¹¤×÷Çø - ±àÒëÉÕÂ¼ |

Ê¹ÓÃ `tools\sync.bat` Í¬²½Ô´Âëµ½ ADS ¹¤×÷Çø¡£
# AURIX TC377 new_ins é¡¹ç›®é…ç½®

> é¡¹ç›®: new_ins (AURIX TC377 æƒ?æ€§å?¼èˆªç³»ç»Ÿ)
> ADS ç‰ˆæœ¬: v1.10.2
> èŠ?ç‰?: TC377TP

---

## é¡¹ç›®æ¦‚è¿°

æœ?é¡¹ç›®æ˜?åŸºäºŽ Infineon AURIX TC377 çš„æƒ¯æ€§å?¼èˆªç³»ç»Ÿ (INS)ï¼ŒåŒ…å?ï¼?
- EKF èžåˆå§¿æ€ä¼°è®?
- ç”µæœº/ç¼–ç å™¨æŽ§åˆ?
- GNSS/GPS æŽ¥æ”¶
- æ— çº¿æ•°æ®ä¼ è¾“

---

## é‡è?çº¦æ?

### ç¼–ç çº¦æŸ
**C/C++ æºæ–‡ä»¶ä½¿ç”? GB2312 ç¼–ç ï¼?**
- `.c` / `.h` æ–‡ä»¶ä½¿ç”¨ GB2312
- `.md` / `.json` / `.xml` ç­‰ä¿æŒ? UTF-8
- Hook å·²é…ç½?è‡?åŠ¨è½¬æ? UTF-8 â†? GB2312ï¼ˆæŽ’é™? .md/.json ç­‰ï¼‰

### æž„å»ºçº¦æŸ
**å¿…é¡»ä½¿ç”¨ ADS IDE æž„å»ºï¼?**
- ä¿?æ”¹ä»£ç åŽï¼Œåœ¨ ADS IDE ä¸?æŒ? Ctrl+B æž„å»º
- ä¸èƒ½ä½¿ç”¨å‘½ä»¤è¡Œç¼–è¯? (Tasking éžå•†ä¸šç‰ˆé™åˆ¶)
- æž„å»ºæˆåŠŸåŽï¼Œå?ç”? `tools\headless_build.bat --flash-only` çƒ§å½•

---

## å¿…è?»æ–‡æ¡?

åœ¨å¼€å§‹å·¥ä½œå‰ï¼Œè?·å…ˆé˜…è?»ä»¥ä¸‹æ–‡æ¡£ï¼š

1. **[å·¥å…·é“¾çº¦æŸæŒ‡å—](docs/TOOLCHAIN_GUIDE.md)** - äº†è§£ä½•æ—¶ä½¿ç”¨ä»€ä¹ˆå·¥å…?
2. **[é¡¹ç›®ç´¢å¼•](docs/PROJECT_INDEX.md)** - äº†è§£é¡¹ç›®ç»“æž„å’Œæ¨¡å?
3. **[å®žçŽ°æ‘˜è?](docs/INS_IMPLEMENTATION_SUMMARY.md)** - INS å®žçŽ°ç»†èŠ‚

---

## å…³é”®è·?å¾?

```
code/
â”œâ”€â”€ system/init_all.c    # æ‰€æœ‰å?–è?¾åˆå§‹åŒ–
â”œâ”€â”€ system/interrupt.c   # ä¸?æ–?ä»»åŠ¡è°ƒåº¦
â”œâ”€â”€ ins/Ins.c           # INS æ ¸å¿ƒå¼•æ“Ž
â””â”€â”€ control/            # ç”µæœº/PID/ç¼–ç å™?

tools/headless_build.bat  # æž„å»º+çƒ§å½•è„šæœ¬
scripts/serial_reader.py   # ä¸²å£æ•°æ®é‡‡é›†
Debug/new_ins.elf         # å›ºä»¶æ–‡ä»¶
```

---

## å¸¸ç”¨æ“ä½œ

| æ“ä½œ | å‘½ä»¤ |
|------|------|
| çƒ§å½•å›ºä»¶ | `tools\headless_build.bat --flash-only` |
| é‡‡é›†ä¸²å£æ•°æ® | `python scripts\serial_reader.py -t 60 --save` |
| åˆ†æžæ¼‚ç§» | `python scripts\analyze_drift.py data\file.txt` |

---

## è°ƒè¯•ç«?å?

- **COM33**: Debug Port (Infineon DAS JDS) - ç”¨äºŽçƒ§å½•å’Œè°ƒè¯?
- **COM24**: Wireless Data Port (USB-SERIAL, 115200bps) - æ•°æ®è¾“å‡º
