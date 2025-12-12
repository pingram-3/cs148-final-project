import pandas as pd
import matplotlib.pyplot as plt

nodes_expanded_data = {
    'Path': ['Short Path', 'Medium Path', 'Long Path'],
    'Dijkstra': [2746, 4597, 7355],
    'A*': [51, 72, 104],
    'Greedy Best First Search': [52, 76, 106]
}

maximum_queue_size_data = {
    'Path': ['Short Path', 'Medium Path', 'Long Path'],
    'Dijkstra': [75, 94, 103],
    'A*': [104, 146, 210],
    'Greedy Best First Search': [106, 154, 213]
}

order = ['Short Path', 'Medium Path', 'Long Path']
nodes_df = pd.DataFrame(nodes_expanded_data)
queue_df = pd.DataFrame(maximum_queue_size_data)

nodes_df.set_index('Path').reindex(order).plot(kind='bar', stacked=True, ylabel='Number of Nodes Expanded', title='Number of Nodes Expanded', rot=0).set_yscale('log')
queue_df.set_index('Path').reindex(order).plot(kind='line', title='Maximum Queue Size', ylabel='Maximum Queue Size', rot=0).set_yscale('log')
plt.tight_layout()
plt.show()