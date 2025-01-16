#!/usr/bin/env python3

"""This launcher simply start Webots."""

import optparse
import subprocess

optParser = optparse.OptionParser()
optParser.add_option("--world", dest="world", default="", help="Path to the world to load.")
optParser.add_option("--mode", dest="mode", default="realtime", help="Startup mode.")
optParser.add_option("--no-gui", dest="noGui", default="false", help="Start Webots with minimal GUI.")

options, args = optParser.parse_args()

command = [ 'webots', '--mode=' + options.mode, options.world]
if options.noGui.lower() == 'true':
    command.append('--stdout')
    command.append('--stderr')
    command.append('--batch')
    command.append('--no-rendering')
    command.append('--minimize')

subprocess.call(command)
