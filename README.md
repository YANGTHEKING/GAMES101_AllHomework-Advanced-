# GAMES101 作业记录

这个仓库用于记录 GAMES101 的作业实现。官方模板和个人作业分开保存，方便从原始框架开始重新编译，也方便对照自己的修改。

## 目录结构

```text
.
├── 官方模板/
│   ├── 作业0/
│   ├── 作业1/
│   ├── ...
│   └── 作业8/
├── 作业0/ ... 作业8/    # 个人作业记录
└── 大作业/              # 个人大作业记录
```

`官方模板/` 中的内容来自课程官方发布的作业压缩包，按作业编号归档；`作业0/` 至 `作业8/` 和 `大作业/` 保留个人实现与运行记录。模板目录不包含本仓库的作业答案。

## 官方模板来源

| 作业 | 官方下载 | 说明 |
| --- | --- | --- |
| 作业 0 | [pa0.zip](https://games-cn.org/wp-content/uploads/2020/02/pa0.zip) | 基础环境与向量练习 |
| 作业 1 | [Assignment1.zip](https://games-cn.org/wp-content/uploads/2020/02/Assignment1.zip) | 变换与光栅化 |
| 作业 2 | [Assignment2.zip](https://games-cn.org/wp-content/uploads/2020/03/Assignment2.zip) | 深度测试与纹理 |
| 作业 3 | [Assignment3.zip](https://games-cn.org/wp-content/uploads/2020/03/Assignment3.zip) | 光栅化器 |
| 作业 4 | [Assignment4.zip](https://games-cn.org/wp-content/uploads/2020/03/Assignment4.zip) | 贝塞尔曲线与反走样 |
| 作业 5 | [Assignment5.zip](https://games-cn.org/wp-content/uploads/2020/03/Assignment5.zip) | 光线与三角形求交 |
| 作业 6 | [PA6.zip](https://games-cn.org/wp-content/uploads/2020/03/PA6.zip) | 光线追踪 |
| 作业 7 | [PA7-1.zip](https://games-cn.org/wp-content/uploads/2020/04/PA7-1.zip) | 路径追踪 |
| 作业 8 | [assignment8.zip](https://games-cn.org/wp-content/uploads/2020/04/assignment8.zip) | 质子映射 |

课程主页：[GAMES101: 现代计算机图形学入门](https://sites.cs.ucsb.edu/~lingqi/teaching/games101.html)。作业 3 的框架在课程论坛中有过修正版说明，使用时请同时参考[作业公告](https://games-cn.org/forums/topic/graphics-intro-hwN/)。

## 如何使用

每个模板或个人作业目录按照其中的 `README`、作业 PDF 和 `CMakeLists.txt` 操作。以 CMake 项目为例：

```bash
cmake -S <作业目录> -B <作业目录>/build
cmake --build <作业目录>/build
```

不同作业的依赖和运行参数可能不同，请以对应目录中的说明为准。仓库会忽略后续生成的 `build/`、CMake 缓存和编译输出。

## 课程资料与说明

个人学习笔记和补充说明：[CSDN 文章目录](https://blog.csdn.net/ycrsw/category_11717021.html)。

课程资料对作业框架和作业答案的公开发布有相应要求。本仓库保留官方来源链接并将模板与个人实现分开，请在使用、修改和再发布时遵守课程及原作者的发布规则。
