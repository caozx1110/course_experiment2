#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File   :   config.py
@Time   :   2022/10/19 22:45:01
@Author :   Cao Zhanxiang 
@Version:   1.0
@Contact:   caozx1110@163.com
@License:   (C)Copyright 2022
@Desc   :   config
'''

import os
os.chdir(os.path.dirname(__file__))

from munch import DefaultMunch
import yaml

config_dict = yaml.load(open('./config.yaml', 'r'), Loader=yaml.FullLoader)
config = DefaultMunch.fromDict(config_dict)

if __name__ == '__main__':
    print(type(config.wifi.ssid))
    print(config.wifi.ssid)
    pass
    # print(type(config['camera']['R']))