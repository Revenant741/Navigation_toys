#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random
import math
import copy

show_animation = True

class RRT():
    """
    RRTによる経路計画
    """

    def __init__(self, start, goal, obstacleList,
                 randArea, expandDis=1.0, goalSampleRate=5, maxIter=5000):
        """
        パラメータ設定
        start:初期座標 [x,y]
        goal:目標座標 [x,y]
        obstacleList:障害物のリスト [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]
        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=True):
        """
        経路計画
        animation: flag for animation on or off
        """
        self.nodeList = [self.start]
        while True:

            # ランダムサンプリング
            # ゴールとなる座標を与えているのは、高速化のため？
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # rndに最も近いノードのインデックスを求める
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # 最も近いノードとの角度差を計算
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
            #print(rnd)

            # 最も近いノードの座標からrnd座標に向かって距離がself.expandDisとなる座標を計算
            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)

            # 最も近いノードのインデックス番号を親ノードとする
            newNode.parent = nind

            # 障害物との衝突判定、衝突しない場合には続行
            if not self.__CollisionCheck(newNode, nearestNode, self.obstacleList):
                continue

            # nodeListの末端に新しいノードを追加
            self.nodeList.append(newNode)
            print("nNodelist:", len(self.nodeList))

            # 新しいノードの座標とゴールの座標がself.expandDis以下であれば終了
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

            if animation:
                self.DrawGraph(rnd)

        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1

        # ゴールのノードからスタートのノードをつなげ、pathに座標を代入する
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent

        path.append([self.start.x, self.start.y])

        return path

    def DrawGraph(self, rnd=None):

        i = 0
        plt.clf()
        ax = plt.axes()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], label=i)
                i = i + 1
                if i <= 10:
                    i = 0

        for (ox, oy, w, h) in self.obstacleList:
            r = patches.Rectangle(xy=(ox, oy), width=w, height=h, fc='#000000', fill=True)
            ax.add_patch(r)
            #plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        #座標rndと各ノードの座標のユークリッド距離を計算
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        #ユークリッド距離が最も小さいノードのインデックス番号を返す
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node1, node2, obstacleList):
        for (ox, oy, w, h) in obstacleList:
            # 矩形の座標(最後の要素は矩形のはじまりの座標)を代入
            xlist = [ox, ox + w, ox + w, ox, ox]
            ylist = [oy, oy, oy + h, oy + h, oy]

            # 線分と矩形の接触判定
            # 実際には2本の線分(線分と矩形を構成する１辺)の接触判定を行う
            # 線分abの設定
            ax, ay = node1.x, node1.y
            bx, by = node2.x, node2.y
            for i in range(4):
                # 線分abと線分cdの設定
                cx, cy = xlist[i], ylist[i]
                dx, dy = xlist[i + 1], ylist[i + 1]

                ta = (cx - dx) * (ay - cy) + (cy - dy) * (cx - ax)
                tb = (cx - dx) * (by - cy) + (cy - dy) * (cx - bx)
                tc = (ax - bx) * (cy - ay) + (ay - by) * (ax - cx)
                td = (ax - bx) * (dy - ay) + (ay - by) * (ax - dx)

                if(tc * td < 0 and ta * tb <= 0):
                    return False

        return True

class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def main():
    print("経路計画開始")

    #障害物の設定
    obstacleList = [
        (-1, 1, 2, 5),
        (3, 4, 4, 2),
        (6, 8, 2, 4),
        (10, 0, 2, 6),
    ]  # [x,y,w,h]


    #パラメータの設定
    rrt = RRT(start=[0, 0], goal=[12, 12],
              randArea=[-2, 15], obstacleList=obstacleList)
    path = rrt.Planning(animation=show_animation)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.savefig('result.png')
        plt.show()

if __name__ == '__main__':
    main()