#!/usr/bin/env python3

import sys

from rqt_gui.main import Main as RQT_Main

from evaluation_nodes.evaluation_plugin import EvaluationPlugin

if __name__ == "__main__":
    plugin = "evaluation_plugin"
    main = RQT_Main(filename=plugin)

    sys.exit(main.main(standalone=plugin))
