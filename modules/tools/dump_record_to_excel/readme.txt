工具使用步骤：
1, 在yaml文件中定义需要解析的topic以及具体的属性
2, 运行脚本

详细说明：
第一步：
	# 在/apollo/modules/tools/dump_record/dump_record.yaml中添加需要解析的topic信息
	# name即topic的名称，与cyber monitor中看到的名称一致
	# keyword即想要提取这个topic中具体哪个属性信息，keyword采取模糊匹配，只要属性中包含关键字就会被记录下来。
	# keyword留空则表示记录该topic下的所有信息,但是keyword关键字必须保留

	# 注意：
	# 1, 目前脚本不支持数量可变的属性提取，比如障碍物的数量每次都不一样，即在cyber monitor中可以用右键扩展的属性目前不支持
	# 2, yaml语法要求，冒号后面必须接一个空格，然后再写具体信息，空格不可省略

	# 示例 -----------------------------------------------------

	Topic:
	  - name: /apollo/planning   # 提取planning这个topic中包含point和rss关键字的所有属性
		keyword:
		  - point
		  - rss

	  - name: /apollo/control    # 提取control这个topic中包含speed关键字的所有属性
		keyword:
		  - speed

	  - name: /apollo/prediction  # 提取prediction这个topic中的所有属性
		keyword:

第二步：
	进入apollo容器内，运行 ./scripts/dump_record.sh 「-s 1635927111.6681252」 「-d 5」 record文件路径  文件输出目录
	其中-s和-d是可选参数，分别表示开始时间（start_time）和持续时间（duration）。详情请查看程序或运行 ./scripts/dump_record.sh -h
	开始时间（start_time）可以通过cyber_record play 播包时获取，本程序也会打印出record文件的最早的时间
	举例：
	./scripts/dump_record.sh  data/record/1103.record  data/record/test  -d  5  -s  1635927115.909
