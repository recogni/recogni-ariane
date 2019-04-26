from tools.edalize.edalize import *
import os
import json

def gen_source_listing(json_file):
    files = list()
    include_dirs = list()
    with open(json_file) as json_file:
        # parse file names
        data = json.load(json_file)
        for group in data:
            for file in group['files']:
                is_include = False
                if file.endswith(".sv"):
                    file_type = "systemVerilogSource"
                elif file.endswith(".svh"):
                    file_type = "systemVerilogSource"
                    is_include = True
                elif file.endswith(".v"):
                    file_type = "systemVerilogSource"
                elif file.endswith(".vhd"):
                    file_type = "vhdlSource"
                else:
                    break
                files.append({
                    'name': file,
                    'file_type': file_type,
                    'is_include_file': is_include
                })
            for include_dir in group['include_dirs']:
                include_dirs.append(include_dir)
    return include_dirs, files

work_root = 'work'
tool = 'modelsim'
include_dirs, files = gen_source_listing('sim.json')
parameters = {}
args = []
# print(files)
edam = {
  'files'        : files,
  'name'         : 'ariane',
  'parameters'   : parameters,
  'toplevel'     : 'ariane_tb',
  'tool_options' : {
    'modelsim': {
    'vlog_options' : ["-suppress 13314", "-suppress 2583"]
    }
  },
}

backend = get_edatool(tool)(edam=edam,
                            work_root=work_root)

os.makedirs(work_root, exist_ok=True)
backend.configure(args)

backend.build()

# unknown
# cSource
# cppSource
# asmSource
# vhdlSource
# vhdlSource-87
# vhdlSource-93
# verilogSource
# verilogSource-95
# verilogSource-2001
# swObject
# swObjectLibrary
# vhdlBinaryLibrary
# verilogBinaryLibrary
# unelaboratedHdl
# executableHdl
# systemVerilogSource
# systemVerilogSource-3.0
# systemVerilogSource-3.1
# systemCSource
# systemCSource-2.0
# systemCSource-2.0.1
# systemCSource-2.1
# systemCSource-2.2
# veraSource
# eSource
# perlSource
# tclSource
# OVASource
# SVASource
# pslSource
# systemVerilogSource-3.1a
# SDC
# vhdlAmsSource
# verilogAmsSource
# systemCAmsSource
# libertySource
# user
