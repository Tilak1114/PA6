import math


def dijikstra(adj_list, start_node):
    dij_res = {}
    visited = {}

    # (cost, parentx, parenty)

    queue = [start_node]

    iter = 1

    for k in adj_list.keys():
        cost = math.inf
        par = None
        if k == start_node:
            cost = 0

        dij_res[k] = [cost, par]
        visited[k] = False

    while queue:
        print("")
        print("Iter ", iter)

        curr_node = queue[0]

        print("Parent ", curr_node)

        neighbors = adj_list[curr_node]

        visited[curr_node] = True

        queue.pop(0)

        true_vis = []
        for k, v in visited.items():
            if v:
                true_vis.append(k)
        print("Visited ", true_vis)

        for n in neighbors:
            if curr_node == n[0]:
                continue

            n, n_c = n

            parent_node_cost = dij_res[curr_node][0]
            current_neigh_cost = n_c

            if parent_node_cost + current_neigh_cost <= dij_res[n][0]:
                dij_res[n][0] = parent_node_cost + current_neigh_cost
                dij_res[n][1] = curr_node

            if not visited[n]:
                queue.append(n)
                visited[n] = True
        iter += 1

        print("Openset ", queue)

        print("Dijikstra Result", dij_res)

    return dij_res


adj_list = {}
dont_include = [8, 9, 10, 11, 13, 16, 17, 18, 19, 23, 30, 34, 35, 36, 37]


def get_cost(i, j, ni, nj):
    if (
            (ni == i - 1 and nj == j - 1) or
            (ni == i - 1 and nj == j + 1) or
            (ni == i + 1 and nj == j - 1) or
            (ni == i + 1 and nj == j + 1)
    ):
        return 1.5
    elif i == ni and j == nj:
        return 0
    else:
        return 1


for i in range(6):
    for j in range(8):
        node_id = i * 8 + j
        if node_id in dont_include:
            continue
        for n_i in range(max(0, i - 1), min(6, i + 2)):
            for n_j in range(max(0, j - 1), min(8, j + 2)):
                if n_i * 8 + n_j in dont_include:
                    continue

                cost = get_cost(i, j, n_i, n_j)

                if node_id in adj_list:
                    adj_list[node_id].append((n_i * 8 + n_j, cost))
                else:
                    adj_list[node_id] = [(n_i * 8 + n_j, cost)]

print(adj_list)

dij_res = dijikstra(adj_list, 40)
