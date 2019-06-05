#ifndef LOCAL_CACHED_INFO_H_
#define LOCAL_CACHED_INFO_H_
#include <vector>
#include <cmath>
using namespace std;
namespace localMap
{
unsigned char NO_INFORMATION = 255;
unsigned char LETHAL_OBSTACLE = 254;
unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
unsigned char FREE_SPACE = 0;
float resolution_ = 0.05;     //地图分辨率
float inscribed_radius_ = 0.25;//机器人内切圆半径
float weight_ = 10;
int radius = 11;//半径50cm，5cm栅格10格。
float dist;
vector<int>* computeCostTable()
{
 vector<int>* table = new vector<int>(256,0);
 return table;
}

vector<vector<int> >* computeCachedCost()
{
 vector<vector<int> >* cachedCost = new vector<vector<int> >(radius+1,vector<int>(radius+1,0));
 return cachedCost;
}


vector<vector<float> >* computeCachedDistance()
{
 vector<vector<float> >* cachedDist = new vector<vector<float> >(radius+1,vector<float>(radius+1,0));
 return cachedDist;
}

vector<vector<float> >* cached_distance = computeCachedDistance();
vector<vector<int> >* cached_cost = computeCachedCost();
vector<int>* cost_translation_table = computeCostTable();
inline unsigned char computeCost(double distance)
{
 unsigned char cost = 0;
 if (distance == 0)
    cost = LETHAL_OBSTACLE;
  else if (distance * resolution_ <= inscribed_radius_)
    cost = INSCRIBED_INFLATED_OBSTACLE;
  else
  {
    // make sure cost falls off by Euclidean distance
    double euclidean_distance = distance * resolution_;
    double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
    cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
   }
    return cost;
}
void computeCaches()
{
 for(int i = 0; i <= radius; ++i)
 {
  for(int j = 0; j <= radius; ++j)
  {
   dist = hypot(i,j);
   (*cached_distance)[i][j] = dist;
   (*cached_cost)[i][j] = computeCost(dist);
  }
 }
 (*cost_translation_table)[0] = 0;  // NO obstacle
 (*cost_translation_table)[253] = 99;  // INSCRIBED obstacle
 (*cost_translation_table)[254] = 100;  // LETHAL obstacle
 (*cost_translation_table)[255] = -1;  // UNKNOWN

  for (int i = 1; i < 253; i++)
 {
  (*cost_translation_table)[i] = int(1 + (97 * (i - 1)) / 251);
 }
}



class CellData
{
public:
  CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
     index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy){}
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};
}


#endif  
