#include "boids.h"

Boids::Boids(size_t totalBoids, float minSpeed, float maxSpeed, const int boxSize)
    : totalBoids(totalBoids), h_position(std::vector<float3>(totalBoids)), h_velocity(std::vector<float3>(totalBoids))
{
    // cpu memory
    generateRandomPositions(boxSize);
    generateRandomVelocities(minSpeed, maxSpeed);
    
    // gpu memory
    allocateDeviceMemory();
    cudaMemcpy(d_velocity, h_velocity.data(), h_velocity.size() * sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_position, h_position.data(), h_position.size() * sizeof(float3), cudaMemcpyHostToDevice);
}

Boids::~Boids()
{
    cudaFree(d_position);
    cudaFree(d_velocity);
}

void Boids::updatePositions(float deltaTime)
{
    for (int i = 0; i < totalBoids; i++)
    {
        h_position[i].x = h_position[i].x + h_velocity[i].x * deltaTime;
        h_position[i].y = h_position[i].y + h_velocity[i].y * deltaTime;
        h_position[i].z = h_position[i].z + h_velocity[i].z * deltaTime;
    }
}

float Boids::calculateDistance(int centerBoid, int otherBoid)
{
    auto posC = h_position[centerBoid],
        posO = h_position[otherBoid];
    return glm::length(glm::vec3(posC.x, posC.y, posC.z) - glm::vec3(posO.x, posO.y, posO.z));
}

void Boids::allocateDeviceMemory()
{
    cudaMalloc((void**)&d_position, h_position.size() * sizeof(float3));
    cudaMalloc((void**)&d_velocity, h_velocity.size() * sizeof(float3));
}

void Boids::generateRandomPositions(const int boxSize)
{
    // https://stackoverflow.com/questions/288739/generate-random-numbers-uniformly-over-an-entire-range
    std::cout << "[BOIDS] Generating random positions" << std::endl;
    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    std::uniform_real_distribution<float> distr(0.0f, boxSize);

    for (size_t i = 0; i < totalBoids; i++)
    {
        h_position[i].x = distr(generator);
        h_position[i].y = distr(generator);
        h_position[i].z = distr(generator);
    }
    std::cout << "Done!" << std::endl;
}

void Boids::generateRandomVelocities(float minSpeed, float maxSpeed)
{
    // https://stackoverflow.com/questions/288739/generate-random-numbers-uniformly-over-an-entire-range
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<float> dirDist(-1.0f, 1.0f);
    std::uniform_real_distribution<float> speedDist(minSpeed, maxSpeed);

    std::cout << "[BOIDS] Generating random velocities" << std::endl;
    for (int i = 0; i < totalBoids; i++) 
    {
        float3 dir = make_float3(dirDist(generator), dirDist(generator), dirDist(generator));
        float length = sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);

        if (length > 1e-6f) {
            dir.x /= length;
            dir.y /= length;
            dir.z /= length;
        }
        else 
        {
            dir = make_float3(1.0f, 0.0f, 0.0f);
        }

        float speed = speedDist(generator);
        auto vel = make_float3(dir.x * speed, dir.y * speed, dir.z * speed);
        h_velocity[i] = vel;
    }
    std::cout << "Done!" << std::endl;
}