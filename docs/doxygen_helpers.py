#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""doxygen_helpers.py
"""
import sys
import os
import pathlib
import subprocess
import jinja2
import yaml
from urllib.parse import quote
from urllib.parse import unquote
from xml.etree import ElementTree as ET
# from jinja2 import Environment, PackageLoader, select_autoescape
# env = Environment(loader=PackageLoader("doxygen"),
#                   autoescape=select_autoescape())

CATEGORIES = (
    ('core', 'cyber', 'cyber'),
    ('core', 'planning', 'modules/external_command'),
    ('core', 'planning', 'modules/routing'),
    ('core', 'planning', 'modules/planning'),
    ('core', 'planning', 'modules/storytelling'),
    ('core', 'control', 'modules/control'),
    ('core', 'canbus', 'modules/canbus'),
    ('core', 'canbus', 'modules/canbus_vehicle'),
    ('core', 'perception', 'modules/perception'),
    ('core', 'perception', 'modules/third_party_perception'),
    ('core', 'prediction', 'modules/prediction'),
    ('core', 'drivers', 'modules/drivers'),
    ('core', 'localization', 'modules/localization'),
    ('core', 'dreamview', 'modules/dreamview'),
    ('core', 'dreamview', 'modules/dreamview_plus'),
    ('core', 'task_manager', 'modules/task_manager'),
    ('core', 'monitor', 'modules/monitor'),
    ('core', 'guardian', 'modules/guardian'),
    ('core', 'tools', 'modules/tools'),
    ('core', 'audio', 'modules/audio'),
    ('core', 'bridge', 'modules/bridge'),
    ('core', 'common', 'modules/common'),
    ('core', 'common', 'modules/common_msgs'),
    ('core', 'contrib', 'modules/contrib'),
    ('core', 'v2x', 'modules/x2x'),
    ('core', 'transform', 'modules/transform'),
    ('core', 'calibration', 'modules/calibration'),
    ('core', 'data', 'modules/data'),
    ('core', 'map', 'modules/map'),
)


def remove_prefix(text, prefix):
    """remove_prefix
    """
    if text.startswith(prefix):
        return text[len(prefix):]
    return text


def remove_suffix(text, suffix):
    """remove_suffix
    """
    if text.endswith(suffix):
        return text[:-len(suffix)]
    return text


def doxylink_encode(path):
    """doxylink_encode
    """
    rules = (
        ('.md', ''),
        ('_', '\\_\\_'),
        ('.', '_8'),
        ('/', '_2'),
        ('+', '_09'),
        (' ', '_01'),
    )
    result = path
    for rule in rules:
        result = result.replace(*rule)

    return result


def doxylink_encode_unescape(path):
    """doxylink_encode
    """
    rules = (
        ('.md', ''),
        ('_', '__'),
        ('.', '_8'),
        ('/', '_2'),
        ('+', '_09'),
        (' ', '_01'),
    )
    result = path
    for rule in rules:
        result = result.replace(*rule)

    result = quote(result).replace('%', '_x')
    return result


class Package():
    """Package"""

    def __init__(self, path):
        """__init__

        Args:
            path (str): package path

        Returns:
            None

        """
        self.path = path
        self.cyberfile = ET.parse(os.path.join(path, 'cyberfile.xml'))
        self.category = ''

    @property
    def name(self):
        """name
        """
        return self.cyberfile.findtext('name')

    @property
    def version(self):
        """version
        """
        return self.cyberfile.findtext('version')

    @property
    def depends(self):
        """depends
        """
        return [x.text for x in self.cyberfile.findall('depend')]

    @property
    def readme_cn_path(self):
        """readme_cn_path
        """
        return pathlib.Path(self.path).joinpath('README_cn.md')

    @property
    def readme_en_path(self):
        """readme_en_path
        """
        return pathlib.Path(self.path).joinpath('README.md')

    @property
    def readme_path(self):
        """readme_path
        """
        if self.readme_cn_path.exists():
            return self.readme_cn_path
        if self.readme_en_path.exists():
            return self.readme_en_path
        return self.readme_cn_path

    def readme_md_link(self):
        """readme_md_link
        """
        return f'[{self.name}]({self.readme_path})'

    def readme_md_doxylink(self):
        """readme_md_doxylink
        """
        encoded_path = doxylink_encode(self.readme_path.as_posix())
        return f'@subpage md_{encoded_path} "{self.name}"'

    def readme_md_doxylink_ref(self):
        """readme_md_doxylink_ref
        """
        encoded_path = doxylink_encode_unescape(self.readme_path.as_posix())
        return f'@ref md_{encoded_path}'


def find_packages(prefix: str):
    """find packages in prefix
    """
    for root, _, files in os.walk(prefix):
        if 'cyberfile.xml' in files:
            yield Package(root)
            continue


def generate_packages_cn_md():
    """generate_packages_cn_md
    """
    categories = CATEGORIES
    pkg_core_cn_md_path = 'docs/packages/packages_core_cn.md'
    output = [
        r'# 包',
        r'',
        f'@subpage md_{doxylink_encode(pkg_core_cn_md_path)} "Apollo Core"',
        r'',
        r'|仓库|分类|包名|',
        r'|---|---|---|',
    ]
    for repo, category, path in categories:
        for pkg in find_packages(path):
            if pkg.readme_path.exists():
                output.append(f'|{repo}|{category}|{pkg.readme_md_link()}|')

    prettier_output = subprocess.check_output(
        ['prettier', '--parser=markdown'],
        input='\n'.join(output).encode('utf-8'),
    )
    print(prettier_output.decode('utf-8'))


def generate_packages_core_cn_md():
    """generate_packages_core_cn_md
    """
    categories = CATEGORIES
    output = [
        r'# Apollo Core',
        r'',
        r'|仓库|分类|包名|',
        r'|---|---|---|',
    ]
    for repo, category, path in categories:
        for pkg in find_packages(path):
            if pkg.readme_path.exists():
                output.append(
                    f'|{repo}|{category}|{pkg.readme_md_doxylink()}|')

    prettier_output = subprocess.check_output(
        ['prettier', '--parser=markdown'],
        input='\n'.join(output).encode('utf-8'),
    )
    print(prettier_output.decode('utf-8'))


def render_doxygen_tree_item_html(item):
    """render_doxygen_tree_item_html
    """
    link = ''
    if len(item[1]) > 0:
        link = f'@ref md_{doxylink_encode_unescape(item[1])}'
    if len(item) >= 3 and len(item[2]) > 0:
        return '\n'.join([
            f'<tab type="usergroup" visible="yes" title="{item[0]}" intro="" url="{link}">',
            *[render_doxygen_tree_item_html(i) for i in item[2]], r'</tab>'
        ])
    return f'<tab type="user" visible="yes" title="{item[0]}" intro="" url="{link}" />'


def generate_treeview_html():
    """generate_treeview_html
    """
    tree_data = (
        ('发版说明', 'docs/RELEASE_cn.md'),
        ('安装说明', '', (
            ('必备软件安装指南',
             'docs/installation_instructions/essential_software_installation_guide_cn.md'
             ),
            ('快速开始', 'docs/installation_instructions/quick_start_cn.md'),
        )),
        ('使用指南', '', (
            ('包管理使用教程', 'docs/user_guidelines/packages_tutorial_cn.md'),
            ('核心模块', '', (
                ('感知能力', '', (
                    ('功能综述', '', (
                        ('模块概述',
                         'docs/user_guidelines/modules/perception/overview/overview_cn.md'
                         ),
                        ('功能列表',
                         'docs/user_guidelines/modules/perception/overview/function_list_cn.md'
                         ),
                    )),
                    ('开发模式', '', (
                        ('参数开发',
                         'docs/user_guidelines/modules/perception/development/development_params_cn.md'
                         ),
                        ('插件开发',
                         'docs/user_guidelines/modules/perception/development/development_plugins_cn.md'
                         ),
                        ('新增模型',
                         'docs/user_guidelines/modules/perception/development/development_models_cn.md'
                         ),
                        ('组件开发',
                         'docs/user_guidelines/modules/perception/development/development_components_cn.md'
                         ),
                    )),
                    ('模块解析', '',
                     (('框架设计',
                       'docs/user_guidelines/modules/perception/analysis/framework_design_cn.md'
                       ), )),
                )),
                ('规划能力', '', (
                    ('功能综述', '', (
                        ('模块概述',
                         'docs/user_guidelines/modules/planning/overview/overview_cn.md'
                         ),
                        ('功能列表',
                         'docs/user_guidelines/modules/planning/overview/function_list_cn.md'
                         ),
                    )),
                    ('开发模式', '', (
                        ('概述',
                         'docs/user_guidelines/modules/planning/development/overview_cn.md'
                         ),
                        ('通过发布命令开发',
                         'docs/user_guidelines/modules/planning/development/development_commands_cn.md'
                         ),
                        ('通过新增插件开发',
                         'docs/user_guidelines/modules/planning/development/development_plugins_cn.md'
                         ),
                        ('通过配置参数开发',
                         'docs/user_guidelines/modules/planning/development/development_params_cn.md'
                         ),
                    )),
                    ('模块解析', '', (
                        ('目录结构',
                         'docs/user_guidelines/modules/planning/analysis/directory_structure_cn.md'
                         ),
                        ('框架设计',
                         'docs/user_guidelines/modules/planning/analysis/framework_design_cn.md'
                         ),
                    )),
                )),
            )),
            ('可视化交互工具 Dreamview+', '', (
                ('Dreamview+ 概述',
                 'docs/user_guidelines/dreamview_plus/dreamview_plus_overview_cn.md'
                 ),
                ('Add Panel',
                 'docs/user_guidelines/dreamview_plus/add_panel_cn.md'),
                ('Mode Settings',
                 'docs/user_guidelines/dreamview_plus/mode_settings_cn.md'),
                ('Resource Manager',
                 'docs/user_guidelines/dreamview_plus/resource_manager_cn.md'),
                ('PNC可视化调试', '', (
                    ('场景仿真',
                     'docs/user_guidelines/dreamview_plus/pnc_visual_debugging/scenario_simulation_cn.md'
                     ),
                    ('复制全部坐标点信息',
                     'docs/user_guidelines/dreamview_plus/pnc_visual_debugging/copy_coordinate_point_information_cn.md'
                     ),
                    ('测距',
                     'docs/user_guidelines/dreamview_plus/pnc_visual_debugging/distance_measurement_cn.md'
                     ),
                    ('自由仿真',
                     'docs/user_guidelines/dreamview_plus/pnc_visual_debugging/free_simulation_cn.md'
                     ),
                    ('轨迹绘制',
                     'docs/user_guidelines/dreamview_plus/pnc_visual_debugging/trajectory_drawing_cn.md'
                     ),
                )),
            )),
            ('研发工具', '', (
                ('Apollo 研发工具 - aem',
                 'docs/user_guidelines/tools/tool_aem_cn.md'),
                ('Apollo 研发工具 - buildtool',
                 'docs/user_guidelines/tools/tool_buildtool_cn.md'),
            )),
        )),
        ('应用实践', '', (('上机实践教程', '', (
            ('Apollo 感知实践（2.0）', '', (
                ('基于插件进行开发',
                 'docs/practices/tutorial/perception_practical_2.0/development_based_on_plugins_cn.md'
                 ),
                ('基于组件进行开发',
                 'docs/practices/tutorial/perception_practical_2.0/development_based_on_components_cn.md'
                 ),
                ('如何在感知模式下播包',
                 'docs/practices/tutorial/perception_practical_2.0/how_to_play_bag_in_perception_mode_cn.md'
                 ),
            )),
            ('Apollo 规划实践（2.0）', '', (
                ('左转待转场景仿真调试',
                 'docs/practices/tutorial/planning_practical_2.0/left_turn_waiting_scene_simulation_debugging_cn.md'
                 ),
                ('红绿灯场景仿真调试',
                 'docs/practices/tutorial/planning_practical_2.0/traffic_light_scene_simulation_debugging_cn.md'
                 ),
                ('限速区域场景仿真调试',
                 'docs/practices/tutorial/planning_practical_2.0/speed_limit_area_scene_simulation_debugging_cn.md'
                 ),
            )),
        )), )),
        ('常见问题', '',
         (('Dreamview+', '',
           (('如何把英文界面切换为中文',
             'docs/faqs/dreamview_plus/how_to_switch_english_to_chinese_cn.md'
             ), )), )),
    )
    for item in tree_data:
        print(render_doxygen_tree_item_html(item))

    categories = CATEGORIES
    for repo, category, path in categories:
        for pkg in find_packages(path):
            if pkg.readme_path.exists():
                print(
                    f'<tab type="user" visible="yes" title="{pkg.name}" intro="" url="{pkg.readme_md_doxylink_ref()}" />'
                )


def generate_dir_html(dirpath, prefix='', indent_level=0):
    """generate_dir_html
    """
    dir_title = dirpath.split('/')[-1]
    indent = '    ' * indent_level
    # print(indent + dirpath)
    print(
        f'{indent}<tab type="usergroup" visible="yes" title="{dir_title}" intro="" url="">'
    )
    for name in os.listdir(dirpath):
        if name.startswith('.'):
            continue
        if os.path.isdir(os.path.join(dirpath, name)):
            generate_dir_html(os.path.join(dirpath, name), prefix,
                              indent_level + 1)
        elif os.path.isfile(os.path.join(dirpath,
                                         name)) and name.endswith('.md'):
            filepath = os.path.join(dirpath, name)
            sub_indent = '    ' * (indent_level + 1)
            title = remove_suffix(name, '.md')
            # print(sub_indent + os.path.join(dirpath, name))
            print(
                f'{sub_indent}<tab type="user" visible="yes" title="{title}" intro="" url="@ref md_{doxylink_encode_unescape(filepath)}" />'
            )
    # for root, dirs, files in os.walk(dirpath):
    #     print('root', root)
    #     print('dirs', dirs)
    #     print('files', files)
    #     for filename in files:
    #         if filename.endswith('.md'):
    #             filepath = os.path.join(root, filename)
    #             sub_indent = '    ' * (indent_level + 1)
    #             title = remove_suffix(filename, '.md')
    #             print(sub_indent + os.path.join(root, filename))
    #             # print(
    #             #     f'{sub_indent}<tab type="user" visible="yes" title="{title}" intro="" url="@ref md_{doxylink_encode_unescape(filepath)}" />'
    #             # )
    #     print(2)
    #     for dirname in dirs:
    #         generate_dir_html(os.path.join(root, dirname), prefix,
    #                           indent_level + 1)

    print(f'{indent}</tab>')


def generate_treeview_html2():
    """generate_treeview_html2
    """
    dirs1 = (
        'docs/发版说明',
        'docs/安装指南',
        'docs/应用实践',
    )
    for dirent in dirs1:
        generate_dir_html(dirent)
    dirs4 = ('docs/框架设计', )
    dirs2 = ('docs/工具使用', )
    for dirent in dirs2:
        generate_dir_html(dirent, 'docs')
    # 源代码文档


def scan_dir(dirpath, indent_level=0):
    """scan_dir
    """
    print(f'{indent_level}\t{dirpath}/')
    for name in os.listdir(dirpath):
        if name.startswith('.'):
            continue
        if os.path.isdir(os.path.join(dirpath, name)):
            scan_dir(os.path.join(dirpath, name), indent_level + 1)
        elif os.path.isfile(os.path.join(dirpath,
                                         name)) and name.endswith('.md'):
            filepath = os.path.join(dirpath, name)
            print(f'{indent_level + 1}\t{filepath}')


def scan_docs_dir():
    """scan_docs_dir
    """
    dirs1 = (
        'docs/发版说明',
        'docs/安装指南',
        'docs/应用实践',
        'docs/框架设计',
        'docs/工具使用',
    )
    for dirent in dirs1:
        scan_dir(dirent)


def generate_treeview_html3():
    """generate_treeview_html3
    """
    with open('./docs.structure.txt', 'r') as fin:
        docs_structure = list(
            map(lambda x: x.split('\t'),
                fin.read().splitlines()))
    output = []
    stack = []
    last_level = 0
    for (level, name) in docs_structure:
        level = int(level)
        if level < last_level:
            for _ in range(last_level - level):
                stack.pop()
                indent = '    ' * len(stack)
                output.append(f'{indent}</tab>')
        if name.endswith('/'):
            title = name.split('/')[-2]
            indent = '    ' * int(level)
            # print(f'{indent}{name}')
            output.append(
                f'{indent}<tab type="usergroup" visible="yes" title="{title}" intro="" url="">'
            )
            stack.append(name)
        elif name.endswith('.md'):
            title = remove_suffix(name.split('/')[-1], '.md')
            indent = '    ' * int(level)
            # print(f'{indent}{name}')
            output.append(
                f'{indent}<tab type="user" visible="yes" title="{title}" intro="" url="@ref md_{doxylink_encode_unescape(name)}" />'
            )
            upper_dir = name.split('/')[-2]
            if upper_dir == '核心模块':
                for line in generate_treeview_modules_html_old():
                    output.append(line)
        last_level = level

    while len(stack) > 0:
        stack.pop()
        indent = '    ' * len(stack)
        output.append(f'{indent}</tab>')

    # print('\n'.join(output))
    with open('./docs/DoxygenLayout.xml.tpl', 'r') as fin:
        template = jinja2.Template(fin.read())
        print(template.render(docs_structure_html='\n'.join(output)))
    return output


class PageItem():
    """PageItem"""

    def __init__(self, **attrs):
        """__init__

        Args:
            title (str): title

        Returns:
            None

        """
        self.title = attrs.get('title', '')
        self.intro = attrs.get('intro', '')
        self.type = attrs.get('type', '')
        self.prefix = attrs.get('prefix', '')
        self._url = attrs.get('url', '')
        self.pages = []
        if 'pages' in attrs:
            for page in attrs['pages']:
                if type(page) in (str, bytes):
                    self.pages.append(
                        PageItem(prefix=pathlib.Path(self.prefix).joinpath(
                            self.title).as_posix(),
                                 title=page))
                else:
                    self.pages.append(
                        PageItem(prefix=pathlib.Path(self.prefix).joinpath(
                            self.title).as_posix(),
                                 **page))

        if 'path' in attrs:
            self.path = pathlib.Path(attrs['path'])

    @property
    def url(self):
        """url
        """
        if self._url:
            return self._url
        if hasattr(self, 'path'):
            encoded_path = doxylink_encode_unescape(self.path.as_posix())
            return f'@ref md_{encoded_path}'
        elif pathlib.Path(self.prefix).joinpath(f'{self.title}.md').exists():
            encoded_path = doxylink_encode_unescape(
                pathlib.Path(
                    self.prefix).joinpath(f'{self.title}.md').as_posix())
            return f'@ref md_{encoded_path}'
        elif len(self.pages) > 0:
            # dir, no url
            return '[none]'
        elif pathlib.Path(self.prefix).joinpath(self.title).is_dir():
            return '[none]'
        return '[none]'

    def html(self, level=0):
        """html
        """
        indent = '  ' * level
        if self.type == 'apollomodules':
            return '\n'.join([(f'{indent}<tab type="usergroup" visible="yes"'
                               f' title="{self.title}"'
                               f' intro="{self.intro}"'
                               f' url="{self.url}">'),
                              *[page.html(level + 1) for page in self.pages],
                              *generate_treeview_modules_html(level + 1),
                              f'{indent}</tab>'])
        if self.type == 'apollopackages':
            return '\n'.join([(f'{indent}<tab type="usergroup" visible="yes"'
                               f' title="{self.title}"'
                               f' intro="{self.intro}"'
                               f' url="{self.url}">'),
                              *[page.html(level + 1) for page in self.pages],
                              *generate_treeview_packages_html(level + 1),
                              f'{indent}</tab>'])
        if len(self.pages) > 0:
            return '\n'.join([(f'{indent}<tab type="usergroup" visible="yes"'
                               f' title="{self.title}"'
                               f' intro="{self.intro}"'
                               f' url="{self.url}">'),
                              *[page.html(level + 1) for page in self.pages],
                              f'{indent}</tab>'])
        return (f'{indent}<tab type="user" visible="yes"'
                f' title="{self.title}"'
                f' intro="{self.intro}"'
                f' url="{self.url}" />')


def generate_treeview_html4():
    """generate_treeview_html4
    """
    with open('./docs/pages_structure.yaml', 'r') as fin:
        structure = yaml.safe_load(fin)

    output = []

    for page in structure.get('pages', []):
        if type(page) in (str, bytes):
            # if page is a string, use it as title
            item = PageItem(prefix='docs', title=page)
            output.append(item.html(2))
            continue
        item = PageItem(prefix='docs', **page)
        output.append(item.html(2))

    # print('\n'.join(output))
    with open('./docs/DoxygenLayout.xml.tpl', 'r') as fin:
        template = jinja2.Template(fin.read())
        print(template.render(docs_structure_html='\n'.join(output)))
    return output


def generate_treeview_packages_html(level=0):
    """generate_treeview_packages_html
    """
    output = []
    pkg_map = {}

    categories = CATEGORIES
    for repo, category, path in categories:
        for pkg in find_packages(path):
            pkg.category = category
            if pkg.readme_path.exists():
                pkg_map.setdefault(category, []).append(pkg)

    indent = '  ' * level
    for category, pkgs in pkg_map.items():
        output.append(f'{indent}<tab type="usergroup" visible="yes"'
                      f' title="{category}" intro="" url="[none]">')
        sub_indent = '  ' * (level + 1)
        for pkg in pkgs:
            output.append(f'{sub_indent}<tab type="user" visible="yes"'
                          f' title="{pkg.name}"'
                          f' intro="" url="{pkg.readme_md_doxylink_ref()}" />')
        output.append(f'{indent}</tab>')

    return output


def generate_treeview_modules_html(level=0):
    """generate_treeview_modules_html
    """
    output = []

    modules = (
        'collection/planning',
        'collection/control',
        'collection/canbus',
        'collection/perception',
        'collection/prediction',
        'collection/localization',
    )

    indent = '  ' * level
    for module_path in modules:
        module = module_path.split('/')[-1]
        module_pkg = Package(f'{module_path}')
        output.append(f'{indent}<tab type="user" visible="yes"'
                      f' title="{module}"'
                      r' intro=""'
                      f' url="{module_pkg.readme_md_doxylink_ref()}" />')

    return output


def generate_treeview_modules_html_old():
    """generate_treeview_modules_html_old
    """
    output = []
    pkg_map = {}

    categories = CATEGORIES
    for repo, category, path in categories:
        for pkg in find_packages(path):
            if pkg.readme_path.exists():
                pkg_map[pkg.name] = pkg

    modules = (
        'collection/planning',
        'collection/control',
        'collection/canbus',
        'collection/perception',
        'collection/prediction',
        'collection/localization',
    )
    for module_path in modules:
        indent = '    ' * 2
        module = module_path.split('/')[-1]
        output.append(
            f'{indent}<tab type="usergroup" visible="yes" title="{module}" intro="" url="">'
        )
        module_pkg = Package(f'{module_path}')
        output.append(
            f'{indent}    <tab type="user" visible="yes" title="概述" intro="" url="{module_pkg.readme_md_doxylink_ref()}" />'
        )
        for dep in module_pkg.depends:
            if dep in pkg_map:
                pkg = pkg_map[dep]
                output.append(
                    f'{indent}    <tab type="user" visible="yes" title="{pkg.name}" intro="" url="{pkg.readme_md_doxylink_ref()}" />'
                )
        output.append(f'{indent}</tab>')

    output.append(
        f'{indent}<tab type="usergroup" visible="yes" title="软件包文档" intro="" url="@ref md_docs_2packages_2packages__cn">'
    )
    output.append(
        f'{indent}    <tab type="usergroup" visible="yes" title="Apollo Core" intro="" url="@ref md_docs_2packages_2packages__core__cn">'
    )
    for pkg in pkg_map.values():
        if pkg.readme_path.exists():
            output.append(
                f'{indent}        <tab type="user" visible="yes" title="{pkg.name}" intro="" url="{pkg.readme_md_doxylink_ref()}" />'
            )
    output.append(f'{indent}    </tab>')
    output.append(f'{indent}</tab>')

    return output


def main():
    """main entry
    """
    command = 'generate_packages_cn_md'
    if len(sys.argv) > 1:
        command = sys.argv[1]

    entrypoint = globals().get(command)
    if callable(entrypoint):
        entrypoint()
    else:
        print(f'Unknown command: {command}')


if __name__ == "__main__":
    sys.exit(main())
