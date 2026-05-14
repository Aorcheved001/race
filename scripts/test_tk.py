#!/usr/bin/env python
# -*- coding: utf-8 -*-
import tkinter as tk

print("Tkinter 测试开始")
root = tk.Tk()
root.title("测试窗口")
root.geometry("400x300")

label = tk.Label(root, text="Hello World - Tkinter 可用!")
label.pack(pady=50)

btn = tk.Button(root, text="关闭", command=root.destroy)
btn.pack()

print("窗口创建成功，3秒后自动关闭...")
root.after(3000, root.destroy)
root.mainloop()
print("测试完成")
