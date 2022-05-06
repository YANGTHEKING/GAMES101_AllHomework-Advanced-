#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    if(splitMethod == BVHAccel::SplitMethod::NAIVE)
        root = recursiveBuild(primitives);
    else
        root = recursiveSAH(primitives);
    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

double BVHAccel::computeSize(std::vector<Object*> objects){
    Bounds3 centroidBounds;
    for (int i = 0; i < objects.size(); ++i)
        centroidBounds =
            Union(centroidBounds, objects[i]->getBounds());
    return centroidBounds.SurfaceArea();
}

BVHBuildNode* BVHAccel::recursiveSAH(std::vector<Object*> objects){
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveSAH(std::vector{objects[0]});
        node->right = recursiveSAH(std::vector{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        auto beginning = objects.begin();
        auto ending = objects.end();
        if(objects.size() < 12){
            auto middling = objects.begin() +objects.size() / 2;
            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);
            node->left = recursiveSAH(leftshapes);
            node->right = recursiveSAH(rightshapes);
            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        else {
            int bestChoice = 0;
            double minCost = std::numeric_limits<double >::max();
            int bestDim = 0;
            for(int dim = 0; dim < 3; dim++){
                switch (dim) {
                    case 0:
                        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                            return f1->getBounds().Centroid().x <
                                   f2->getBounds().Centroid().x;
                        });
                        break;
                    case 1:
                        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                            return f1->getBounds().Centroid().y <
                                   f2->getBounds().Centroid().y;
                        });
                        break;
                    case 2:
                        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                            return f1->getBounds().Centroid().z <
                                   f2->getBounds().Centroid().z;
                        });
                        break;
                }
                auto l = (float)objects.size();
                float nums[] = {1.0/6, 2.0/6, 3.0/6, 4.0/6, 5.0/6};
                for(int i = 0; i < 5; i++)
                    nums[i] *= l;
                for(int i = 0; i < 5; i++){
                    auto middling = objects.begin() + (int)nums[i];
                    auto leftshapes = std::vector<Object*>(beginning, middling);
                    auto rightshapes = std::vector<Object*>(middling, ending);
                    double leftBoxSize = computeSize(leftshapes);
                    double rightBoxSize = computeSize(rightshapes);
                    double cost = 2.f + (leftBoxSize * leftshapes.size() + rightBoxSize * rightshapes.size()) / bounds.SurfaceArea();
                    if(cost < minCost){   
                        bestChoice = (int)nums[i];
                        bestDim = dim;
                    }
                }
            }
            if(bestDim == 0)
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                        f2->getBounds().Centroid().x;
                });
            if(bestDim == 1)
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                        f2->getBounds().Centroid().y;
                });
            auto middling = objects.begin() + bestChoice;
            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);
            node->left = recursiveSAH(leftshapes);
            node->right = recursiveSAH(rightshapes);
            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        return node;
    }
}

void BVHAccel::quicksort_x(int first, int last, int target, std::vector<Object*> &objects){
    int i = first + 1, j = last;
    while (true) {
		while (j > first && objects[j]->getBounds().Centroid().x >= objects[first]->getBounds().Centroid().x)
			j--;
		while (i < last && objects[i]->getBounds().Centroid().x <= objects[first]->getBounds().Centroid().x)
			i++;
		if (i >= j) 
			break;
		std::swap(objects[i],objects[j]);
	}
	std::swap(objects[j], objects[first]);
    if(j > target)
        quicksort_x(first, j - 1, target, objects);
    if(j < target)
        quicksort_x(j + 1, last, target, objects);
}

void BVHAccel::quicksort_y(int first, int last, int target, std::vector<Object*> &objects){
    int i = first + 1, j = last;
    while (true) {
		while (j > first && objects[j]->getBounds().Centroid().y >= objects[first]->getBounds().Centroid().y)
			j--;
		while (i < last && objects[i]->getBounds().Centroid().y <= objects[first]->getBounds().Centroid().y)
			i++;
		if (i >= j) 
			break;
		std::swap(objects[i],objects[j]);
	}
	std::swap(objects[j], objects[first]);
    if(j > target)
        quicksort_x(first, j - 1, target, objects);
    if(j < target)
        quicksort_x(j + 1, last, target, objects);
}

void BVHAccel::quicksort_z(int first, int last, int target, std::vector<Object*> &objects){
    int i = first + 1, j = last;
    while (true) {
		while (j > first && objects[j]->getBounds().Centroid().z >= objects[first]->getBounds().Centroid().z)
			j--;
		while (i < last && objects[i]->getBounds().Centroid().z <= objects[first]->getBounds().Centroid().z)
			i++;
		if (i >= j) 
			break;
		std::swap(objects[i],objects[j]);
	}
	std::swap(objects[j], objects[first]);
    if(j > target)
        quicksort_x(first, j - 1, target, objects);
    if(j < target)
        quicksort_x(j + 1, last, target, objects);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        int target = objects.size() / 2;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection intersect, intersectl, intersectr;
    if(!node->bounds.IntersectP(ray, ray.direction_inv))
        return intersect;
    if(node->left == nullptr && node->right == nullptr){
        intersect = node->object->getIntersection(ray);
        return intersect;
    }
    intersectl = getIntersection(node->left,ray);
    intersectr = getIntersection(node->right,ray);
    return intersectl.distance < intersectr.distance ? intersectl : intersectr;
}