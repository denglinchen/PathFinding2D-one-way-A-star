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
	
	在本次的开发中，我主要设计了两个类(Node和NodeFactory)来辅助实现寻路算法，该寻路算法
参考经典的A*寻路算法并且根据我自己的理解进行了一些优化和扩展设计，使得该算法使用起来更方
便灵活，可以根据实际业务情况进行调整。
	下面对主要部分进行一些讲解。
	
	Node类：
	首先需要设计一个能保存网格位置Position和计算A*启发式相关信息的路点类——Node，其中需
要保存的信息包括该路点的位置(Position position)、指向前一个路点的指针(Node* parent)、还有
最终的启发值(float f)。
	每一个Node对象只负责保存相关的信息，不具有计算的功能，Node类就像C语言中的Struct一样，
启发值的计算以及position和parent的赋值都由一个工厂对象去完成。是的，我采用了工厂设计模式，
算法的所有细节都由工厂去控制，出了什么问题去找它就可以了。
	
	NodeFactory类：
	路点的工厂是该算法的核心，算法的实现以及其他所有的辅助功能都由该工厂去实现的。那么
FindPath函数体中就只有几行简单的代码了。该工厂主要生产两种“产品”，路点Node对象和路径记
忆vector<Position>，所有的路径记忆则由一个“公共仓库”(static unoredered_map)对象去保存。
	首先，工厂需要保存一些网格地图的信息，它们就是FindPath实参提供的那些信息，Grid const& grid、
Position const starty以及Position const end。它们由构造函数负责初始化。有了这些信息就可以
去构建网格中需要构造的所有路点以及算他们的启发值。

	路点由make_node成员函数构建，需要传入Position和上一个路点的指针Node*两个参数，并且返
回构建好的路点的指针。所有路点都保存在堆上，它们的生命周期和NodeFactory一致，在NodeFactory
的析构函数中将所有路点的内存delete。NodeFactory借助unordered_map<int,unordered_map<int,Node*>> flag
成员保证每一个Postion只会有一个Noded对象，在根据Postion创建Node的时候，如果flag[row][column]
返回了非空的值则表明该路点已经存在，不予创建。使用unordered_map这种数据结构来记录这些信息
既保证了速度也节省了空间，并且可以像二维数组那样使用它。总之工厂对内存的分配十分吝啬。
make_node函数体中的重头戏莫过于启发式中g和h值的计算。具体计算方法查看具体代码的注释。

	add_serround成员函数用来将当前路点的周围路点加入开集合。需要传入当前路点作为Node* parent
和路点开集合的引用multiset<Node*,heuristic> open_set作为参数。由于是八方向的寻路，于是先判
断上下左右四个正方向网格可到达性，并且用四个bool值进行保存，斜方向的四个方向则根据正方向网
格的可达性并且结合IsWalkable来进行判断。最终被判断为可达的路点由make_node来创建，make_node
函数在创建新的路点的时候会顺便将其加入路点开集合。

	get_next函数用于选择下一个前往的路点，并且将被选择的路点从开集合中去除。该函数需要传入
开集合multiset<Node*,heuristic> open_set作为参数，返回下一个路点的指针。需要注意的是，该算
法并没有为闭集合设计容器或者一些标志，而是通过一个特殊的逻辑去判断这个路点是否是闭集合中的
一员。由于NodeFactory保证了每个Position只可以在堆上有一个Node对象，并且在make_node时将其顺便
加入了open_set,所以当一个Node对象的指针被从open_set中剔除以后便被视为加入了闭集，它不会再
被get_next函数查找到，而该Node对象仍然在堆上存活并且履行其职责。get_next通过路点启发值f来选
择最优的路点作为下一个路点返回。启发值f=g*g_d+h*h_d,其中系数g_d和h_d可以通过NodeFactor进行设
置，可以根据实际的业务状况调整两个系数的配比，具体的讨论见PathFindingDll.cpp中的注释。

	fill_path函数则为算法的主要框架和实现，并且负责将waypointList填充实现FindPath的功能。该
函数需要传入waypointList引用作为参数。在fill_path函数中首先会查询路径记忆中存储的已经计算好
的路径，前提是需要开启NodeFactory的记忆功能，如果已经有计算好的路径则直接返回不再进行计算。
否则就进行后续A*的算法的步骤计算路径，再调用make_path生成一个由Node组成的可达路径，最后调用
make_path函数将Position填充进waypointList，如果开启了记忆功能还会将新的记忆vector<Position>
存储进记忆仓库，供下次查询使用。
	
	下面专门介绍一下路径记忆功能的实现：
	一条路径记忆为一个vector<Position>对象，存放在一个四维的unordered_map容器routing_memory中，
根据start和end的row和column值来作为四个key获取。routing_memory是NodeFactory的一个static对象，
其生命周期独立于NodeFactory对象，伴随程序的整个生命周期，因此在每次FindPath被调用的时候以前的
记忆都存在，在find_memory查询到记忆路径的时候，会简单的对路径的有效性做一个判断，就是对路径上
每一个Position调用一次IsWalkable函数，如果发现其中有一个Position不可达，则说明程序的客户加载了
新的地图，旧地图的的路径记忆已经不能再使用，此时会调用clear_memory方法将所有记忆清除。
	如果不想开启记忆功能来节省空间则可以将NodeFactory的bool use_memory设为false，则算法中不会
产生任何有关routing_memory的有效操作。
	同时，还可以通过设置NodeFactory中static int memory_nums和static int max_mem_nums来设置记忆
的最大容量，make_path在每次增加记忆路径之后都会检查这两个值的大小，如果达到上限则会调用void clear_memory
函数来清空记忆。
	
	综上，可以通过NodeFactory的配置控制该寻路算法的性能和行为，具体配置过程参考PathFindingDll.cpp
中的相关注释。

	最后，感谢畅游高校游戏技术竞赛的主办方举办此次竞赛，我在解决这个赛题的过程中学习到了很多宝
贵的知识，从我上面啰嗦的总结就可以看出来！
	再次感谢，祝畅游越办越好！
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
