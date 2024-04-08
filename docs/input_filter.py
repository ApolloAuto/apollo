#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""input filter of doxygen
"""
import os
import pathlib
import re
import sys
import fnmatch
import inspect


def resolve_markdown_linkpath(filename: str, match, strip_prefix=None):
    """resolve file path"""
    fpath = pathlib.Path(filename).parent.joinpath(match.group(2)).resolve()
    if strip_prefix is None:
        strip_prefix = '.'

    matched_path = match.group(2)

    # relative to the file path
    fpath = pathlib.Path(filename).parent.joinpath(matched_path).resolve()
    # absolute path, use project root as the base path
    # TODO: the project root should be configurable, here we use the current
    if matched_path.startswith('/'):
        fpath = pathlib.Path(matched_path).relative_to('/').resolve()
    elif not fpath.exists():
        # relative to the current directory
        fpath = pathlib.Path(matched_path).resolve()

    # get the relative path of project root
    # TODO: the project root should be configurable, here we use the current
    npath = fpath.relative_to(pathlib.Path(strip_prefix).resolve())
    return f'{match.group(1)}{npath.as_posix()}{match.group(5)}'


def process_markdown(filename: str):
    """process markdown file"""
    patt = re.compile(
        r'(\(|\[|"|\')'
        r'((/?[0-9a-zA-Z-_.\u4e00-\u9fa5]+/)*[0-9a-zA-Z-_.\u4e00-\u9fa5]+'
        r'\.(png|jpeg|jpg))'
        r'(\)|\]|"|\')')
    with open(filename, 'r', encoding='utf-8') as fin:
        content = fin.read()
        # if not re.compile(r'^#\s+.*').match(content):
        #     # add title
        #     title = os.path.basename(filename).replace('.md', '')
        #     content = f'# {title}\n\n' + content
        if '[TOC]' not in content:
            # add table of content
            # content = re.sub(r'^(#[^#].*|.*\n=+)', '\\1\n\n[TOC]\n', content)
            content = '[TOC]\n\n' + content

        result = re.sub(patt,
                        lambda x: resolve_markdown_linkpath(filename, x, None),
                        content)
        result = result + (
            '\n\n'
            '## 文档意见反馈\n\n'
            '如果您在使用文档的过程中，'
            '遇到任何问题，请到我们在【开发者社区】建立的 '
            '[反馈意见收集问答页面]'
            '(https://studio.apollo.auto/community/article/163)，'
            '反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。')
        print(result, end='')


def process_proto(filename: str):
    """process proto file"""
    with open(filename, 'r', encoding='utf-8') as fin:
        output = []
        # output.append(f'/** @file {filename} */')
        content = fin.read()
        namespaces = []
        declare_stack = []
        last_stack = []
        for single_line in content.splitlines():
            last_stack.append(single_line)
            line = ''.join(map(lambda x: x.rstrip('\n'), last_stack))
            # print('line:', line, file=sys.stderr)

            match_multiple_line_comment = re.match(r'^\s*/\*.*\*/\s*$', line)
            if match_multiple_line_comment:
                output.append(line)
                last_stack.clear()
                continue

            match_multiple_line_comment_start = re.match(r'^\s*/\*.*$', line)
            if match_multiple_line_comment_start:
                declare_stack.append(('/*', 'comment'))
                output.append(line)
                last_stack.clear()
                continue

            match_multiple_line_comment_end = re.match(r'^.*\*/\s*$', line)
            if match_multiple_line_comment_end:
                if len(declare_stack
                       ) > 0 and declare_stack[-1][1] == 'comment':
                    declare_stack.pop()
                output.append(line)
                last_stack.clear()
                continue

            match_synax = re.match(r'^\s*syntax\s*=\s*"proto.";', line)
            if match_synax:
                for entry in last_stack:
                    output.append(entry)
                last_stack.clear()
                continue

            match_package = re.match(r'^package\s+([a-zA-Z0-9_.]+);', line)
            if match_package:
                namespaces = match_package.group(1).split('.')
                output.append(' '.join(
                    [f'namespace {ns} {{' for ns in namespaces]))
                # for ns in namespaces:
                #     output.append(f'namespace {ns} {{')
                # assume that package statement is in one line
                last_stack.clear()
                continue

            match_enum = re.match(r'^\s*enum\s+([a-zA-Z0-9_]+)\s+{', line)
            if match_enum:
                declare_stack.append((match_enum.group(1), 'enum'))
                # output.append(f'enum {matchEnum.group(1)} {{')
                for entry in last_stack:
                    output.append(entry)
                last_stack.clear()
                continue

            match_message = re.match(r'^\s*message\s+([a-zA-Z0-9_]+)\s+{',
                                     line)
            if match_message:
                declare_stack.append((match_message.group(1), 'message'))
                for entry in f'struct {match_message.group(1)} {{'.split(
                        ' ',
                        len(last_stack) - 1):
                    output.append(entry)
                # output.append(f'struct {matchMessage.group(1)} {{')
                last_stack.clear()
                continue

            match_oneof = re.match(r'^\s*oneof\s+([a-zA-Z0-9_]+)\s+{', line)
            if match_oneof:
                declare_stack.append((match_oneof.group(1), 'oneof'))
                # output.append(f'union {matchOneof.group(1)} {{')
                for entry in f'union {match_oneof.group(1)} {{'.split(
                        ' ',
                        len(last_stack) - 1):
                    output.append(r'  ' * (len(declare_stack) - 1) + entry)
                last_stack.clear()
                continue

            match_closing_brace = re.match(r'^\s*}', line)
            if match_closing_brace:
                output.append('  ' * (len(declare_stack) - 1) + '};')
                if len(declare_stack) > 0:
                    declare_stack.pop()
                last_stack.clear()
                continue

            match_enum_field = re.match(r'^\s*([a-zA-Z0-9_]+)\s*=\s*([0-9]+)',
                                        line)
            if match_enum_field and len(
                    declare_stack) > 0 and declare_stack[-1][1] == 'enum':
                output.append(r'  ' * (len(declare_stack)) +
                              (f'{match_enum_field.group(1)}'
                               r' ='
                               f' {match_enum_field.group(2)},'))
                last_stack.clear()
                continue

            match_message_field = re.match(
                (r'^\s*(optional|required|repeated)?'
                 r'\s*([a-zA-Z0-9_.]+|map<.*?>)'
                 r'\s+([a-zA-Z0-9_]+)\s*=\s*([0-9]+)'
                 r'\s*(\[.*?\])?\s*;(.*)?'), line)
            if match_message_field:
                # print('match_message_field:',
                #       match_message_field.group(1),
                #       match_message_field.group(2),
                #       match_message_field.group(3),
                #       match_message_field.group(4),
                #       file=sys.stderr)
                field_type = match_message_field.group(2).replace('.', '::')
                # field_type = match_message_field.group(2)
                # if field_type:
                #     field_type = field_type.replace('.', '::')
                # else:
                #     field_type = ''
                keyword = match_message_field.group(1)
                if keyword is None:
                    keyword = ''
                field = match_message_field.group(3)
                default_value = match_message_field.group(5)
                if default_value is None:
                    default_value = ''
                output.append(r'  ' * (len(declare_stack)) + (
                    f'{keyword}'
                    f'{" " if keyword else ""}{field_type}'
                    f'{" " if field_type else ""}{field}'
                    ' ='
                    f' {match_message_field.group(4)} {default_value};'
                    f' {match_message_field.group(6)}'))
                last_stack.clear()
                continue

            match_comment = re.match(r'^\s*//', line)
            if match_comment:
                for entry in last_stack:
                    output.append(entry)
                last_stack.clear()
                continue

            match_emptyline = re.match(r'^\s*$', line)
            if match_emptyline:
                for entry in last_stack:
                    output.append(entry)
                last_stack.clear()
                continue

            match_import = re.match(r'^\s*import\s+"([a-zA-Z0-9_./]+)";', line)
            if match_import:
                output.append(f'#include "{match_import.group(1)}"')
                last_stack.clear()
                continue

            if declare_stack and declare_stack[-1][1] == 'comment':
                # in comment block, output orignal line
                for entry in last_stack:
                    output.append(entry)

                last_stack.clear()
                continue
        if namespaces:
            output.append(' '.join(map(lambda x: '}', namespaces)))
            # for ns in reversed(namespaces):
            #     output.append(f'}} // namespace {ns}')
        result = '\n'.join(output)
        print(result, end='')


def process(filename: str):
    """process file"""
    if fnmatch.fnmatch(filename, '*.txt'):
        with open(filename, 'r', encoding='utf-8') as fin:
            print(f'/** @file {filename} */')
            print(fin.read(), end='')

    elif fnmatch.fnmatch(filename, '*.conf'):
        with open(filename, 'r', encoding='utf-8') as fin:
            print(f'/** @file {filename} */')
            print(fin.read(), end='')

    elif fnmatch.fnmatch(filename, '*.proto'):
        process_proto(filename)

    elif fnmatch.fnmatch(filename, '*.md'):
        process_markdown(filename)

    elif not fnmatch.fnmatch(
            filename, os.path.basename(inspect.getfile(
                inspect.currentframe()))):
        with open(filename, 'r', encoding='utf-8') as fin:
            print(fin.read(), end='')


def main():
    """main function
    """
    for filename in sys.argv[1:]:
        process(filename)


if __name__ == '__main__':
    main()
