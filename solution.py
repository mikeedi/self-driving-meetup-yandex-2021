import numpy as np
from collections import defaultdict
from itertools import count


class BaseMap(object):
    def __init__(self, N):
        self.N = N
        pass

    def str_a_to_b(self, a, b):
        raise NotImplementedError()

    def dist(self, a, b):
        raise NotImplementedError()

    def __call__(self, a, b):
        return self.str_a_to_b(a, b)


class Map1(BaseMap):
    """
    map order_with_biggest_costout any obstacles (1 and 2 in tests)
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def str_a_to_b(self, a, b):
        y1, x1 = a
        y2, x2 = b
        x_len = abs(x1 - x2)
        y_len = abs(y1 - y2)
        vertical_dir = 'U' if y2 < y1 else 'D'
        horizontal_dir = 'R' if x2 > x1 else 'L'
        path_str = vertical_dir * y_len + horizontal_dir * x_len
        return path_str

    def dist(self, a, b):
        return len(self.str_a_to_b(a, b))


class Map2(BaseMap):
    """
    map for 3, 4, 5, in tests/
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    # @staticmethod

    def _get_block_center(self, k, m):
        if self.N == 180:
            return 7+12*k, 7+12*m
        if self.N == 384:
            return 13+24*k, 13+24*m
        if self.N == 1024:
            return 17 + 32*k, 17 + 32*m
    # @staticmethod

    def _get_block_num(self, y, x):
        if self.N == 180:
            return (y - 4) // 12, (x - 4) // 12
        if self.N == 384:
            return (y - 7) // 24, (x - 7) // 24
        if self.N == 1024:
            return (y - 9) // 32, (x - 9) // 32

    def str_a_to_b(self, a, b):
        """
        не оптимальный алгоритм - сначала считается до центра кубов, 
        затем прибавляется расстояние между кубами.
        Не оптимально, т.к. роботу придется всегда сначала ехать к центру
        """
        y1, x1 = a
        y2, x2 = b
        block_a = self._get_block_num(y1, x1)
        block_b = self._get_block_num(y2, x2)
        yc1, xc1 = self._get_block_center(*block_a)
        yc2, xc2 = self._get_block_center(*block_b)

        path_str = ''

        # to center a
        vertical_dir = 'U' if yc1 < y1 else 'D'
        horizontal_dir = 'R' if xc1 > x1 else 'L'
        path_str += abs(xc1 - x1)*horizontal_dir + abs(yc1 - y1)*vertical_dir

        # between center
        vertical_dir = 'U' if yc2 < yc1 else 'D'
        horizontal_dir = 'R' if xc2 > xc1 else 'L'
        path_str += abs(xc2 - xc1)*horizontal_dir + abs(yc2 - yc1)*vertical_dir

        # to center b
        vertical_dir = 'U' if y2 < yc2 else 'D'
        horizontal_dir = 'R' if x2 > xc2 else 'L'
        path_str += abs(xc2 - x2)*horizontal_dir + abs(yc2 - y2)*vertical_dir

        return path_str

    def dist(self, a, b):
        return len(self.str_a_to_b(a, b))


class MapInnopolis(BaseMap):
    """docstring for MapInnopolis"""

    def __init__(self, *args):
        super().__init__(*args)

    def str_a_to_b(self, a, b):
        pass

    def dist(self, a, b):
        y1, x1 = a
        y2, x2 = b


class Robot:
    def __init__(self, start_position, _map):
        self.cur_position = start_position
        self.map = _map
        self.prev_work = ''
        self.is_free = True

    def do_order(self, order):
        a, b = order.a, order.b
        path = self.map(self.cur_position, a)
        path += 'T'
        path += self.map(a, b)
        path += 'P'
        self.cur_position = b
        return path, len(path)

    def wait(self, sec):
        return 'S'*sec


class Order:
    _ids = count(0)

    def __init__(self, a, b, max_tips, iteration):
        self.a = a
        self.b = b
        self.iteration_when_added = iteration
        self.max_tips = max_tips
        self.id = next(self._ids)

    def cost_after_finish(self, cur_iteration, path_pre, robot):
        cur_position = robot.cur_position
        total_dist = robot.map.dist(
            cur_position, self.a) + robot.map.dist(self.a, self.b) + 1
        cost = self.max_tips - \
            ((cur_iteration - self.iteration_when_added)*60) - total_dist - path_pre
        return cost

    def get_ab(self):
        return self.a, self.b

    def __repr__(self):
        return f'Order_{self.a}_to_{self.b}'


class Env(object):
    def __init__(self, max_tips, cost, _map, start_position, num_robot=1):
        self.max_tips = max_tips
        self.cost = cost

        # [ for i in range(num_robot)]
        self.robot = Robot(start_position, _map)
        self.orders = []
        self.second_sort_orders = []
        # self.orders = deque()
        self.hash_for_orders = defaultdict(list)
        self.reward = 0
        self.all_a = set()
        # self.damned = []

    def add_order(self, a, b, itr):
        order = Order(a, b, self.max_tips, itr)

        if a in self.all_a:
            self.hash_for_orders[a].append(order)
        else:
            self.all_a.add(a)
            self.orders.append(order)

    def get_order(self, itr, path_pre):
        # рассматриваем только ненулевые заказы
        non_zero = list(filter(lambda x: x.cost_after_finish(
            itr, path_pre, self.robot) > 0, self.orders))
        self.orders = sorted(
            non_zero, key=lambda x: x.cost_after_finish(itr, path_pre, self.robot))
        if len(self.orders) > 0:
            order_with_biggest_cost = self.orders.pop()
            # print(order_with_biggest_cost)
            if len(self.hash_for_orders[order_with_biggest_cost.a]) > 0:
                order = self.hash_for_orders[order_with_biggest_cost.a].pop(0)
                self.orders.append(order)
            return order_with_biggest_cost
        else:
            return None

    def iteration(self, itr):
        path = ""
        while True:
            if not self.robot.is_free:  # если робот еще делает заказ
                # assert len(path) == 0
                left_time = 60
                path += self.robot.prev_work[:left_time]
                self.robot.prev_work = self.robot.prev_work[left_time:]
                if len(self.robot.prev_work) > 0:  # нужно больше, чем одну итерацию
                    # self.robot.prev_work = path[60:]
                    # path = path[:60]
                    break
                else:  # ecли закончил заказ
                    self.robot.is_free = True
                    # assert self.robot.prev_work == ''
                    # self.robot.prev_work = ''

            left_time = 60 - len(path)
            if left_time > 0:
                # self.sort(itr, len(path))
                order = self.get_order(itr, len(path))
                if order is not None:
                    self.reward += order.cost_after_finish(
                        itr, len(path), self.robot)
                    full_path_for_order, full_time_for_order = self.robot.do_order(
                        order)
                    # print(full_time_for_order, full_time_for_order, path)
                    # left_time = 60 - len(path)
                    if full_time_for_order > left_time:
                        path += full_path_for_order[:left_time]
                        # assert self.robot.prev_work == ''
                        self.robot.prev_work += full_path_for_order[left_time:]
                        self.robot.is_free = False
                        break
                    else:
                        path += full_path_for_order

                else:
                    break
            else:
                break

        path += self.robot.wait(60-len(path))
        assert len(path) == 60

        self.print(path)

    def print(self, act):
        global debug
        if debug:
            with open('tmp.txt', 'a') as file:
                print(act, file=file)
        else:
            print(act)


if __name__ == '__main__':
    import sys
    debug = False
    if len(sys.argv) > 1:
        debug = True

    N_to_MAP = {4: Map1, 128: Map1, 180: Map2,
                384: Map2, 1024: Map2, 1000: MapInnopolis}

    N, max_tips, cost = list(map(int, input().split()))
    for _ in range(N):
        input()

    _map = N_to_MAP[N](N=N)
    print(1)
    if N <= 128:
        start_position = (4, 4)
        print(*start_position)
    elif N == 180:
        start_position = (7, 7)  # center of 1st block
        print(*start_position)
    elif N == 384:
        start_position = (13, 13)  # center of 1st block
        print(*start_position)
    elif N == 1024:
        start_position = (17, 17)  # center of 1st block
        print(*start_position)
    else:
        raise NotImplementedError()
    env = Env(max_tips, cost, _map, start_position)

    T, D = list(map(int, input().split()))
    for iteration in range(T):

        num_orders = int(input())
        for i in range(num_orders):
            Srow, Scol, Frow, Fcol = list(map(int, input().split()))
            env.add_order((Srow, Scol), (Frow, Fcol), iteration)
        env.iteration(iteration)

    if debug:
        print('Total reward:', env.reward - cost)
