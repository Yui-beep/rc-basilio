#include <iostream>
#include <cmath>

using namespace std;

int main()
{
	double x1, x2, y1, y2, r1, r2;
	
	cout << "Enter the coordinates of the center of the circle: ";
	cin >> x1 >> y1;
	
	cout << "Enter the radius of the center of the circle 2: ";
	cin >> r1;
	
	cout << "Enter the coordinates of the center of the circle 2: ";
	cin >> x2 >> y2;
	
	cout << "Enter the radius of circle 2: ";
	cin >> r2;
	
	double distancex = (x2 - x1)*(x2 - x1);
	double distancey = (y2 - y1)*(y2 - y1);
	double distance = sqrt(distancex + distancey);
	double sumRadius = r1 + r2;
	
	if (distance < sumRadius)
	{
		cout << "The two circles collided." << endl;
	}
	else
	{
		cout << "The two cicles did not collide." << endl;
	}
	
	return 0;
}