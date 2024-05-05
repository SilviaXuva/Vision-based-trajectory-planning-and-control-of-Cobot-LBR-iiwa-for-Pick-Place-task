from datetime import datetime
import os

class Paths:
    output = os.path.abspath(r'.\Outputs')
    startTime = datetime.now().strftime("%Y-%m-%d_%Hh%Mm%Ss")
    execution = fr'{output}\{startTime}'