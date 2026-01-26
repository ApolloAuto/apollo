#!/usr/bin/env python3
"""
Apollo文档链接更新脚本
用于自动更新MD文档中的内部链接，适配新的页面结构

用法:
    python3 update_md_links.py [directory] [--dry-run] [--verbose]
    
参数:
    directory: 要处理的目录路径，默认为当前目录
    --dry-run: 只显示将要进行的更改，不实际修改文件
    --verbose: 显示详细输出
"""

import os
import re
import argparse
import sys
from pathlib import Path

# 定义更精确的链接映射规则
LINK_MAPPINGS = {
    # 移除开头的docs/前缀（只匹配markdown链接）
    r'\]\(docs/([^)]*\.md)\)': r'](../\1)',
    
    # 更新特定路径模式
    r'\]\(\.\./docs/([^)]*\.md)\)': r'](../\1)',
    
    # 移除单独的docs/前缀（不包含文件扩展名的路径）
    r'\]\(docs/([^)]*)\)': r'](../\1)',
}

def update_links_in_content(content, file_path, verbose=False):
    """更新内容中的链接"""
    original_content = content
    updated_content = content
    
    for pattern, replacement in LINK_MAPPINGS.items():
        updated_content = re.sub(pattern, replacement, updated_content)
    
    if verbose and updated_content != original_content:
        print(f"在文件 {file_path} 中检测到链接需要更新")
        
    return updated_content

def process_md_file(file_path, dry_run=False, verbose=False):
    """处理单个MD文件"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        updated_content = update_links_in_content(content, file_path, verbose)
        
        if content != updated_content:
            if dry_run:
                print(f"[DRY RUN] 需要更新: {file_path}")
                # 显示一些变化的示例
                old_lines = content.split('\n')
                new_lines = updated_content.split('\n')
                for i, (old, new) in enumerate(zip(old_lines, new_lines)):
                    if old != new:
                        print(f"  行 {i+1}:")
                        print(f"    旧: {old}")
                        print(f"    新: {new}")
                        break
            else:
                with open(file_path, 'w', encoding='utf-8') as f:
                    f.write(updated_content)
                print(f"已更新: {file_path}")
                
            return True
        else:
            if verbose:
                print(f"无需更新: {file_path}")
            return False
                
    except Exception as e:
        print(f"处理文件 {file_path} 时出错: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='更新MD文档链接')
    parser.add_argument('directory', nargs='?', default='.', 
                       help='要处理的目录路径')
    parser.add_argument('--dry-run', action='store_true',
                       help='只显示更改，不实际修改文件')
    parser.add_argument('--verbose', action='store_true',
                       help='显示详细输出')
    
    args = parser.parse_args()
    
    target_dir = Path(args.directory)
    
    if not target_dir.exists():
        print(f"错误: 目录不存在: {target_dir}")
        sys.exit(1)
    
    print(f"开始处理目录: {target_dir}")
    print(f"模式: {'预览模式' if args.dry_run else '实际执行'}")
    print("-" * 50)
    
    processed_count = 0
    updated_count = 0
    
    # 遍历所有.md文件
    for md_file in target_dir.rglob("*.md"):
        if md_file.is_file():
            processed_count += 1
            if process_md_file(md_file, args.dry_run, args.verbose):
                updated_count += 1
    
    print("-" * 50)
    print(f"处理完成!")
    print(f"共处理文件: {processed_count}")
    print(f"更新文件: {updated_count}")
    
    if args.dry_run:
        print("\n注意: 这是预览模式，实际文件未被修改。")
        print("要实际应用更改，请移除 --dry-run 参数重新运行。")

if __name__ == "__main__":
    main()
