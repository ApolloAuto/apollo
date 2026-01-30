# 文档链接更新指南

本文档提供了如何更新现有MD文档内部链接的指南，以配合优化的页面结构。

## 推荐的链接格式

### 1. 相对路径链接（推荐）
```markdown
[链接文本](../relative/path/to/file.md)
[Cyber RT概述](../../04_CyberRT/cyber_rt_introduction.md)
```

### 2. 使用路径别名（如适用）
```markdown
[快速导航](快速导航.md)
[安装指南](安装指南/安装指南.md)
```

### 3. 跨目录链接示例

**从应用实践链接到核心模块：**
```markdown
更多信息请参考 [Cyber RT通信机制](../../04_CyberRT/communication_mechanism.md)
```

**从工具使用链接到框架设计：**
```markdown
包管理概念详见 [包管理工具](../../框架设计/软件核心/包管理工具/包管理概念.md)
```

## 需要更新的常见链接模式

### 旧链接模式（需要更新）
```markdown
[旧链接文本](../02_Quick%20Start/old_path.md)
```

### 新链接模式（推荐）
```markdown
[新链接文本](../02_Quick%20Start/new_path.md)
```

## 特殊链接处理

### 1. 图片链接
保持相对路径不变：
```markdown
![图片说明](../02_Quick%20Start/images/image.png)
```

### 2. 外部链接
保持完整URL：
```markdown
[外部资源](https://apollo.baidu.com)
```

## 批量更新建议

建议使用以下工具进行批量链接更新：

### 使用sed命令（Linux/Mac）
```bash
# 更新特定目录下的链接
find docs/ -name "*.md" -exec sed -i 's|docs/02_Quick%20Start/|../02_Quick%20Start/|g' {} \;
```

### 使用Python脚本
```python
import os
import re

def update_links_in_md(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 替换链接模式
    content = re.sub(r'\]\(docs/([^)]+)\)', r'](../\1)', content)
    
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(content)

# 遍历所有md文件
for root, dirs, files in os.walk('docs'):
    for file in files:
        if file.endswith('.md'):
            update_links_in_md(os.path.join(root, file))
```

## 验证链接完整性

更新后，使用以下方法验证链接：

### 1. 使用markdown-link-check
```bash
npx markdown-link-check docs/*.md
```

### 2. 手动检查关键文档
重点检查：
- 主页面（MAINPAGE_cn.md）
- 各章节的概述文档
- 快速导航文档

## 注意事项

1. **保持路径一致性**：确保所有链接使用统一的相对路径格式
2. **避免绝对路径**：不要使用以`/`开头的绝对路径
3. **编码处理**：URL中的空格使用`%20`或直接使用空格
4. **测试验证**：更新后务必验证所有链接的有效性

## 紧急恢复

如果链接更新出现问题，可以：
1. 使用git恢复更改：`git checkout -- docs/`
2. 参考备份的原始链接模式
3. 逐步更新，分段测试
