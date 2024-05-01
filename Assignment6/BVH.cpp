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

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

static Bounds3 getCentroidBounds(const std::vector<Object*>& objects, unsigned begin, unsigned end)
{
    Bounds3 bounds;
    size_t size = std::min(end, static_cast<unsigned>(objects.size()));
    for (unsigned i = begin; i < size; ++i)
    {
        Object* object = objects[i];
        if (!object)
        {
            continue;
        }
        bounds = Union(bounds, object->getBounds().Centroid());
    }
    return bounds;
}

static unsigned getMiddle(BVHAccel::SplitMethod splitMethod, const std::vector<Object*>& objects)
{
    unsigned size = objects.size();
    if (splitMethod != BVHAccel::SplitMethod::SAH)
    {
        return size / 2;
    }

    unsigned mid = 1u;
    double time_max = std::numeric_limits<double>::max();
    unsigned num_buckets = 31;
    for (unsigned i = 1; i < num_buckets; ++i)
    {
        unsigned i_mid = size * i / num_buckets;
        Bounds3 leftBounds = getCentroidBounds(objects, 0, i_mid);
        Bounds3 rightBounds = getCentroidBounds(objects, i_mid, size);
        double leftArea = leftBounds.SurfaceArea();
        double rightArea = rightBounds.SurfaceArea();
        double totalArea = leftArea + rightArea;
        double inv_totalArea = 1.0 / totalArea;

        double time = leftArea * inv_totalArea * i_mid + rightArea * inv_totalArea * (size - i_mid);
        if (time < time_max)
        {
            time_max = time;
            mid = i_mid;
        }
    }
    return mid;
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    if (objects.empty())
    {
        return nullptr;
    }

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

        unsigned middle = getMiddle(splitMethod, objects);
        auto beginning = objects.begin();
        auto middling = objects.begin() + middle;
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
    // Traverse the BVH to find intersection
    if (!node)
    {
        return {};
    }

    const Vector3f& dir = ray.direction;
    if (!node->bounds.IntersectP(ray, ray.direction_inv, {dir.x < 0, dir.y < 0, dir.z < 0}))
    {
        return {};
    }

    if (!node->left && !node->right)
    {
        if (!node->object)
        {
            return {};
        }
        return node->object->getIntersection(ray);
    }

    Intersection hit1 = getIntersection(node->left, ray);
    Intersection hit2 = getIntersection(node->right, ray);
    if (hit1.distance <= hit2.distance)
    {
        return hit1;
    }
    return hit2;
}