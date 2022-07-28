#!/usr/bin/env python

from message import messages__

mes = messages__()

mes.set_Cones_count_all(255)
tmp = mes.Cones_count_all
while tmp['length'] > 0:
    print(tmp['data'] & 0xff)
    tmp['length']-=8
    tmp['data'] = tmp['data'] >> 8
