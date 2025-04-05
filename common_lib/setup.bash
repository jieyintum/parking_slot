#!/bin/bash
cur_dir=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
#echo $cur_dir
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${cur_dir}/lib/x86
