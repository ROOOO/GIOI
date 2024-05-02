//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // Implement Path Tracing Algorithm
    Intersection inter_shading_point = intersect(ray);
    if (!inter_shading_point.happened || !inter_shading_point.m)
    {
        return {0};
    }

    Vector3f l_dir{ 0.0f };
    Vector3f l_indir{ 0.0f };

    const Vector3f& p = inter_shading_point.coords;
    const Vector3f& wo = ray.direction;
    const Vector3f& N = inter_shading_point.normal;
    Vector3f p_deviation = (dotProduct(ray.direction, N) < 0) ?
                           p + N * EPSILON :
                           p - N * EPSILON ;

    do
    {
        Intersection inter_light;
        float pdf_light = 0.0f;
        sampleLight(inter_light, pdf_light);
        const Vector3f& x = inter_light.coords;
        Vector3f ws = x - p;
        float ws_sqr = dotProduct(ws, ws);
        if (ws_sqr < EPSILON)
        {
            break;
        }

        float dist_light = sqrt(ws_sqr);
        ws = ws / dist_light;
        Ray r(p_deviation, ws);
        Intersection inter_blocked_point = intersect(r);
        if (inter_blocked_point.distance < dist_light - 0.1f)
        {
            break;
        }

        float cos_shading_point = dotProduct(N, ws);
        float cos_light = dotProduct(inter_light.normal, -ws);
        l_dir = inter_light.emit * inter_shading_point.m->eval(wo, ws, N)
                * cos_shading_point * cos_light
                / (ws_sqr * pdf_light);
    } while (false);

    do
    {
        float ksi = get_random_float();
        if (ksi > RussianRoulette)
        {
            break;
        }

        Vector3f wi = inter_shading_point.m->sample(wo, N);
        float wi_sqr = dotProduct(wi, wi);
        if (wi_sqr < EPSILON)
        {
            break;
        }

        float wi_len_inv = 1.0f / sqrt(wi_sqr);
        wi = wi * wi_len_inv;
        Ray r(p_deviation, wi);
        Intersection inter_bounce = intersect(r);
        if (inter_bounce.happened && inter_bounce.obj && !inter_bounce.obj->hasEmit())
        {
            float pdf = inter_shading_point.m->pdf(wo, wi, N);
            if (pdf > EPSILON)
            {
                float cos_shading_point = dotProduct(N, wi);
                l_indir = castRay(r, depth + 1) * inter_shading_point.m->eval(wo, wi, N)
                          * cos_shading_point / (pdf * RussianRoulette);
            }
        }
    } while (false);

    return inter_shading_point.m->getEmission() + l_dir + l_indir;
}