#!/bin/bash

mqtt publish -t 'homeassistant/light/rgb_fairy_lights/bedroom_fairy_lights/config' -h '192.168.100.42' '{"name":"Rainbow Fairy Lights","state_topic":"home/light/qt_py_fairy/power/status","command_topic":"home/light/qt_py_fairy/power/set","brightness_state_topic":"home/light/qt_py_fairy/brightness/status","brightness_command_topic":"home/light/qt_py_fairy/brightness/set","rgb_state_topic":"home/light/qt_py_fairy/rgb/status","rgb_command_topic":"home/light/qt_py_fairy/rgb/set","brightness_scale":100,"optimistic":false}' -r
