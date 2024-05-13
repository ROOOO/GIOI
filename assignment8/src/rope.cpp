#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        if (num_nodes <= 0)
        {
            return;
        }

        Vector2D step(0.0f, 0.0f);
        if (num_nodes > 1)
        {
            step = (end - start) / (num_nodes - 1);
        }
        for (int i = 0; i < num_nodes; ++i)
        {
            Vector2D position = start + i * step;
            masses.push_back(new Mass(position, node_mass, false));
            if (i > 0)
            {
                springs.push_back(new Spring(masses[i - 1], masses[i], k));
            }
        }
        for (auto &i : pinned_nodes)
        {
            if (i >= num_nodes || i < 0)
            {
                continue;
            }
            if (!masses[i])
            {
                continue;
            }
            masses[i]->pinned = true;
        }
    }

    static void addForces(Spring* s)
    {
        if (!s || !s->m1 || !s->m2)
        {
            return;
        }

        Vector2D v = s->m2->position - s->m1->position;
        double v_len = v.norm2();
        if (v_len < 1e-10)
        {
            return;
        }

        v_len = sqrt(v_len);
        Vector2D force = s->k * v / v_len * (v_len - s->rest_length);
        s->m1->forces += force;
        s->m2->forces -= force;
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // Use Hooke's law to calculate the force on a node
            addForces(s);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;

                // Add global damping
                float k_d = 0.01f;
                m->forces += -k_d * m->velocity;

                Vector2D a = m->forces / m->mass;

                // explicit method
                // m->position += m->velocity * delta_t;
                // m->velocity += a * delta_t;

                // semi-implicit method
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // Simulate one timestep of the rope using explicit Verlet （solving constraints)
            addForces(s);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // Set the new position of the rope mass
                m->forces += gravity * m->mass;

                // Add global Verlet damping
                Vector2D a = m->forces / m->mass;

                float damping_factor = 0.0001f;
                m->position += (1.0f - damping_factor) * (temp_position - m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
