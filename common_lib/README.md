# ***ReleaseNote***

## 2022.11.18
### CemLog
1.   bug-fix 级别设置为DEFAULT时有丢前面1秒的日志

## 2022.11.17
### LoadParameter
* http://10.94.119.180:8090/display/PPCEMALG/LoadParameter

## 2022.5.19
### CemLog
1.   提供通过环境变量`CEMLOG_${CONTEXTSTR}_LEVEL`动态调整打印等级的特性
2.   缩短默认日志头中文件名信息中的路径信息部分，只保留文件名

## 2022.5.16
### CemLog
#### 封装DLT日志
1.   通过封装，提供print和std::cout类似形式的接口
2.   提供简化后的初始化函数，去除繁琐配置，同时也保留可以自己DIY配置的方法
