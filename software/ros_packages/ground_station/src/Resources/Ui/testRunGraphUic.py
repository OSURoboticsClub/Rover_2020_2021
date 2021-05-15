import test_graph_uic as graph
import time
from PyQt5.QtCore import QTimer,QDateTime

Graph = graph.main()
while True:
    timer = QTimer()
    timer.timeout.connect(Graph.plot())
    timer.start(500)