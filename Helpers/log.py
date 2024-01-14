from datetime import datetime
import json
import numpy as np
import os
import sys
# import PySimpleGUI as sg # py -m pip install PySimpleGUI

class Paths:
    output = os.path.abspath(r'.\Outputs')
    execution = fr'{output}\{datetime.now().strftime("%Y-%m-%d_%Hh%Mm%Ss")}'
    os.makedirs(execution, exist_ok=True)

class Logger(object):
    def __init__(self, file):
        self.terminal = sys.stdout
        self.file = open(file, 'w', encoding = 'utf-8')
   
    def __call__(self, *message):
        message = list(message)
        for i, msg in enumerate(message):
            if isinstance(msg, np.ndarray):
                message[i] = np.array2string(msg, formatter={'float_kind':lambda x: "%.5f" % x})
            elif isinstance(msg, list):
                message[i] = ', '.join([str(m) for m in msg])
            elif isinstance(msg, dict):
                msg = {(k):(np.array2string(v, formatter={'float_kind':lambda x: "%.5f" % x}) if isinstance(v, np.ndarray) else v) for k,v in msg.items()}
                message[i] = json.dumps(msg, indent=4)
            elif isinstance(msg, float):
                message[i] = str(float("{:.5f}".format(msg)))
            else:
                try:
                    message[i] = str(msg)
                except Exception as e:
                    pass
        print('\n'.join(message))
        self.file.write('\n'.join(message) + '\n')
        self.file.flush()
        os.fsync(self.file.fileno())
        # sg.Print('\n'.join(message) + '\n', size=(30,28), location=(304, 182))

Log = Logger(fr'{Paths.execution}\output.log')