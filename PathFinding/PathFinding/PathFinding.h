#ifndef IKPGAJDGHOFEKABFDAKEFLIFBAEBBKAK
#define IKPGAJDGHOFEKABFDAKEFLIFBAEBBKAK

#include <cstdlib>
#include <cstdint>
#include<math.h>
#include<vector>
#include<unordered_map>
#include<algorithm>
#include<set>
using namespace std;

class Position
{
public:
  std::size_t row;
  std::size_t column;
};

class Grid
{
public:
  std::uint8_t const * walkability;
  std::size_t numRows;
  std::size_t numColumns;

  bool IsWalkable(Position const & pos) const
  {
    return 0 == walkability[pos.row * numColumns + pos.column];
  }
};

class IWaypointList
{
public:
  virtual ~IWaypointList()
  {

  }

  virtual void Add(Position const &) = 0;
};

/////////////////////////////////////////////////////////////////////////////////////////
/*                                  ***开发总结***
参赛者：邓琳琛
学校：北京航空航天大学
电话：15910922135
邮箱：515143675@qq.com
完成日期：2020年7月29日
visual studio版本：Visual Studio 2019（v142） Debug x86
*/

//路点
class Node
{
public:
	//该路点的坐标
	Position position;
	//启发值
	float f;
	//上一个路点
	Node* parent;
};

//用来创建路点对象的工厂类
class PathBuilder
{
public:
	//路径记忆
		//如果起点和终点重复输入可以直接获取上一次计算好的路径,空间换时间
		//和程序有一样的生命周期，而NodeFactory的生命周期只在FindPath函数体内。
	static unordered_map<int, unordered_map<int, unordered_map<int, unordered_map<int, vector<Position>>>>> routing_memory;
	static int memory_nums;
	static int max_mem_nums;
	//是否启用路径记忆功能
		//取决于内存是否足够
	bool use_memory;

	//启发式系数设定
		//路径代价项的系数
	float h_d;
		//终点距离项系数
	float g_d;

	//保存寻路需要的信息
	Grid const& grid;
	Position const start;
	Position const end;
	//记录被创建过的点
		//在该点集合中存在并且在开集合nodes中不存在的路点则视为闭集
	unordered_map<int,unordered_map<int,Node*>> flag;

	struct heuristic
	{
		bool operator()(Node* const& node1, const Node* const& node2)const
		{
			//将Node按启发值从小到大排序
			return node1->f < node2->f;
		}
	};
	//路点集合（开集合）
	multiset<Node*,heuristic> open_set;//插入取出复杂度均为logn，优于vector的n复杂度

	PathBuilder(Grid const &grid_, Position const &start_, Position const &end_,bool use_mem=true)
		:grid(grid_), start(start_), end(end_),use_memory(use_mem)
	{
		if (use_memory == false)
			clear_memory();
		//启发项系数默认为1
		h_d = 1.0;
		g_d = 1.0;
	}

	//创建路点
	Node* make_node(Position const p,Node* parent)
	{
		/*
		Debug日志：
			FindPath接口貌似会传入一些超出grid范围的position
			在这里使用一个if判断将这样的坏点过滤掉可以解决这个Bug
		*/
		//检查坏点
		if (p.column < grid.numColumns && p.row < grid.numRows)
		{
			//每个路点只能被创建一次
			if (flag[p.row][p.column] != nullptr)
				return nullptr;

			Node* this_node = new Node();
			this_node->position = p;

			this_node->parent = parent;

			/****启发式设计相关代码****/
			//路径代价g的计算
			float g = 0;
			Node* iter = this_node;
			while (iter->parent != nullptr)
			{
				//斜边路径代价
				if(abs(int(iter->position.column-iter->parent->position.column))
					+abs(int(iter->position.row - iter->parent->position.row))==2)
					g += sqrtf(2);
				//直角边路径代价
				else
				{
					g += 1;
				}
				iter = iter->parent;
			}

			//距离项h的计算
			float h = 0;
				//由于使用八方向网格，g采用切比雪夫确定，并且假定斜边距离为直角边的根号2倍
			float dx = abs(float(float(this_node->position.column) - float(end.column)));
			float dy = abs(float(float(this_node->position.row) - float(end.row)));
			h = (dx+dy)+ (sqrtf(2) - 2) * min(dx, dy);

			//启发值
			this_node->f = g * g_d + h * h_d;
			/****启发式设计相关代码****/
			
			flag[p.row][p.column] = this_node;
			return this_node;
		}
		else
		{
			return nullptr;
		}
		
	}

	//添加相邻路点，并且加入开集合
	void add_serround(Node* parent, multiset<Node*,heuristic> &open_set)
	{
		bool up_flag=false,down_flag=false,right_flag=false,left_flag=false;
		//上下左右的添加
		//添加上路点
		Position tmp = parent->position;
		tmp.row -= 1;
		if (tmp.row >= 0 && grid.IsWalkable(tmp))
		{
			up_flag = true;
			Node* this_node = make_node(tmp, parent);
			if (this_node != nullptr)open_set.insert(this_node);
		}
		//添加下路点
		tmp = parent->position;
		tmp.row += 1;
		if (tmp.row < grid.numRows && grid.IsWalkable(tmp))
		{
			down_flag = true;
			Node* this_node = make_node(tmp, parent);
			if (this_node != nullptr)open_set.insert(this_node);
		}
		//添加左路点
		tmp = parent->position;
		tmp.column -= 1;
		if (tmp.column >= 0 && grid.IsWalkable(tmp))
		{
			left_flag = true;
			Node* this_node = make_node(tmp, parent);
			if (this_node != nullptr)open_set.insert(this_node);
		}
		//添加右路点
		tmp = parent->position;
		tmp.column += 1;
		if (tmp.column < grid.numColumns && grid.IsWalkable(tmp))
		{
			right_flag = true;
			Node* this_node = make_node(tmp, parent);
			if (this_node != nullptr)open_set.insert(this_node);
		}

		//斜边点的添加
		//添加左上
		tmp = parent->position;
		tmp.row -= 1;
		tmp.column -= 1;
		if (up_flag && left_flag && grid.IsWalkable(tmp))
		{
			Node* this_node = make_node(tmp, parent);
			if (this_node != nullptr)open_set.insert(this_node);
		}
		//添加右上
		tmp = parent->position;
		tmp.row -= 1;
		tmp.column +=1 ;
		if (right_flag && up_flag && grid.IsWalkable(tmp))
		{
			Node* this_node = make_node(tmp, parent);
			if (this_node != nullptr)open_set.insert(this_node);
		}
		//添加右下
		tmp = parent->position;
		tmp.column += 1;
		tmp.row += 1;
		if (right_flag && down_flag&&grid.IsWalkable(tmp))
		{
			Node* this_node = make_node(tmp, parent);
			if (this_node != nullptr)open_set.insert(this_node);
		}
		//添加左下
		tmp = parent->position;
		tmp.column -= 1;
		tmp.row += 1;
		if (left_flag && down_flag&&grid.IsWalkable(tmp))
		{
			Node* this_node = make_node(tmp, parent);
			if (this_node != nullptr)open_set.insert(this_node);
		}
	}

	//从开集合中获取下一个遍历的路点并且将其从开集合中删除（视为加入闭集）
	Node* get_next(multiset<Node*,heuristic>& open_set)
	{
		//在开集合中寻找待删除的路点
		Node* next = nullptr;
		//从二叉堆中取出最小启发值的Node作为next返回
		if (open_set.size() > 0)
		{
			next = *open_set.begin();
			open_set.erase(open_set.begin());
		}
		
		//将被删除的点返回,当成新的parent
		return next;
	}

	//获取记忆
	bool find_memory(IWaypointList& waypointList)
	{
		vector<Position> mem;
		//查询到有记忆
		if (routing_memory[start.row][start.column][end.row][end.column].size() > 0)
		{
			mem = routing_memory[start.row][start.column][end.row][end.column];
			//检测地图是否更新过
			for (Position& p : mem)
			{
				if (grid.IsWalkable(p) == false)
				{
					//检测到更新，清除记忆重新计算
					clear_memory();
					return false;
				}
			}
			//没有更新过地图,正常填充
			for (Position& p : mem)
				waypointList.Add(p);
			return true;
		}
		//没有记忆
		return false;
		
	}

	//填充路径和记忆路径
	void make_path(IWaypointList& waypointList, Node* end_node)
	{
		vector<Node*> nodes;
		while (end_node != nullptr)
		{
			nodes.push_back(end_node);
			end_node = end_node->parent;
		}
		if (nodes.size() == 0)return;
		reverse(nodes.begin(), nodes.end());
		vector<Position> new_routing;
		for (int i = 0; i < nodes.size(); ++i)
		{
			waypointList.Add(nodes[i]->position);
			if (use_memory)
				new_routing.push_back(nodes[i]->position);
		}
		if (use_memory)
		{
			routing_memory[start.row][start.column][end.row][end.column] = new_routing;
			++memory_nums;
			//检查记忆数量 超过限制就清除
			if (memory_nums > max_mem_nums)
				clear_memory();
		}

	}	

	//清除记忆的api
	void clear_memory()
	{
		memory_nums = 0;
		routing_memory.clear();
	}

	//获取终点路点
	Node* get_end_node()
	{
		return flag[end.row][end.column];
	}

	void fill_path(IWaypointList& waypointList)
	{
		//先查找记忆
		if (use_memory&&find_memory(waypointList))
			return;

		//起始路点
		Node* start_node =make_node(start, nullptr);
		//剔除坏值的影响
		if (start_node == nullptr)return;

		//添加起点周围的路点
		add_serround(start_node, open_set);
		//获得下一个parent路点
		Node* next = get_next(open_set);

		//当有路可走时循环
		while (next!=nullptr)
		{
			//是否走到了终点
			if (next->position.column == end.column && next->position.row == end.row)
				break;

			//将新的周围路点加入开集合
			add_serround(next, open_set);
			//选择下一个前往的路点，并且将next路点从开集合去除（视为加入闭集）
			next =get_next(open_set);
		}
	
		//如果没有解
		Node* end_node = get_end_node();
		if (end_node == nullptr)
			return;
	
		//如果有解next就是end_node
		//添加路径
		make_path(waypointList, end_node);
	}

	//释放所有创建过的路点
	~PathBuilder()
	{
		for (auto it_1 = flag.begin(); it_1 != flag.end(); ++it_1)
		{
			for (auto it_2 = (*it_1).second.begin(); it_2 != (*it_1).second.end(); ++it_2)
				delete (*it_2).second;
		}
	}
private:
};

#endif
