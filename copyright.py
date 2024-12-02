import os

# 指定版权信息和 MIT License 内容
COPYRIGHT_TEXT = """
/*
Copyright (c) 2024 loopgad 9th_R2_Member

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
"""

def add_copyright_to_file(file_path):
    """在指定文件开头添加版权信息"""
    with open(file_path, 'r+', encoding='utf-8') as f:
        content = f.read()
        # 检查文件是否已经包含版权信息，避免重复添加
        if COPYRIGHT_TEXT.strip() not in content:
            f.seek(0, 0)
            f.write(COPYRIGHT_TEXT + "\n\n" + content)

def add_copyright_to_directory(directory, file_extensions=None):
    """在指定目录下所有匹配扩展名的文件开头添加版权信息"""
    for root, _, files in os.walk(directory):
        for file_name in files:
            if file_extensions is None or file_name.endswith(tuple(file_extensions)):
                file_path = os.path.join(root, file_name)
                print(f"Adding copyright to: {file_path}")
                add_copyright_to_file(file_path)

# 使用方法
if __name__ == "__main__":
    # 替换为你的目标目录路径
    target_directory = "C:/Users/32806/Desktop/9th_R2/R2_F4_cpp/R2_USER"
    
    # 设置文件扩展名过滤（例如 .py, .cpp, .h），可以自定义或设置为 None
    extensions = ['.py', '.cpp', '.h', '.c']

    # 为目录下所有匹配的文件添加版权信息
    add_copyright_to_directory(target_directory, file_extensions=extensions)
