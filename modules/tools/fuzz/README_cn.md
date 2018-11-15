# 模糊测试

## 介绍

模糊测试通过给被测程序提供随机的或特意构造的输入，并检测程序异常，来发现可能的程序错误，比如内存泄漏。 模糊测试常用于检测软件或系统的安全漏洞。 该目录包括了用于模糊测试的测试用例，每个用例对应了无人车代码的部分逻辑。我们正在持续添加新的测试用例以覆盖更多的无人车功能模块。

如果你有关于如何运行，如何贡献测试用例，或如何提交发现的漏洞，请参考此[教程](https://github.com/BaiduXLab/apollo/blob/master/docs/howto/how_to_write_and_run_fuzz_test.md)

## 编译
```
apollo.sh build_fuzz_test
```

## 运行
```
./bazel-bin/modules/tools/fuzz/[fuzz_test_case] [seed_file]
```

## 种子
种子输入通常是一个对于被测程序正常的输入，它能够帮助模糊测试更快地到达被测程序的核心逻辑，并且能帮助发现更多的程序路径。通常模糊测试需要在高质量的种子输入的帮助下才能达到最好的效果。我们提供当前测试用例所使用的种子输入的[下载](https://github.com/BaiduXLab/libprotobuf-mutator/releases/download/v1.0/apollo_fuzz_seeds.tgz)。
