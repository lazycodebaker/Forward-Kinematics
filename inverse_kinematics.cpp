#include "SFML/Graphics.hpp"
#include "SFML/Window.hpp"
#include "SFML/Main.hpp"
#include "SFML/Config.hpp"
#include "SFML/System.hpp"
#include "SFML/Graphics/PrimitiveType.hpp"
#include "SFML/Graphics/Vertex.hpp"
#include "Eigen/Dense"
#include "vector"
#include "cmath"
#include "iostream"

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const float LINK_LENGTH = 100.0f;
const int NUM_JOINTS = 3;
const float TOLERANCE = 0.1f;
const float DELTA_THETA = 0.01f;
const float SPEED = 0.02f;

struct Link
{
    Eigen::Vector2f start, end;
    float angle;
};

class InverseKinematicsSolver
{
private:
    std::vector<Link> links;
    Eigen::Vector2f target;

public:
    InverseKinematicsSolver(int numLinks)
    {
        links.resize(numLinks);
        float baseX = WINDOW_WIDTH / 2.0f;
        float baseY = WINDOW_HEIGHT / 2.0f;
        for (int i = 0; i < numLinks; ++i)
        {
            links[i].start = {baseX, baseY};
            links[i].end = {baseX, baseY + LINK_LENGTH};
            links[i].angle = 0.0f;
        }
        target = links.back().end;
    }

    void setTarget(const Eigen::Vector2f &newTarget)
    {
        target = newTarget;
    }

    void update()
    {
        Eigen::Vector2f currentEnd = getEndEffector();
        if ((target - currentEnd).norm() < TOLERANCE)
            return;

        for (int i = links.size() - 1; i >= 0; --i)
        {
            Eigen::Vector2f jointToEnd = getEndEffector() - links[i].start;
            Eigen::Vector2f jointToTarget = target - links[i].start;

            float cross = jointToEnd.x() * jointToTarget.y() - jointToEnd.y() * jointToTarget.x();
            float dot = jointToEnd.dot(jointToTarget);
            float errorAngle = atan2(cross, dot);

            links[i].angle += errorAngle * SPEED;
            forwardKinematics();
        }
    }

    void draw(sf::RenderWindow &window)
    {
        for (const auto &link : links)
        {
            std::vector<sf::Vertex> line = {
                {sf::Vector2f(link.start.x(), link.start.y()), sf::Color::White},
                {sf::Vector2f(link.end.x(), link.end.y()), sf::Color::Green}};

            window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);
        };

        sf::CircleShape targetShape(5);
        targetShape.setFillColor(sf::Color::Red);
        targetShape.setPosition(sf::Vector2f(target.x() - 5, target.y() - 5));
        window.draw(targetShape);
    }

private:
    void forwardKinematics()
    {
        Eigen::Vector2f position = links[0].start;
        float cumulativeAngle = 0.0f;

        for (auto &link : links)
        {
            cumulativeAngle += link.angle;
            link.start = position;
            position.x() += LINK_LENGTH * cos(cumulativeAngle);
            position.y() += LINK_LENGTH * sin(cumulativeAngle);
            link.end = position;
        }
    }

    Eigen::Vector2f getEndEffector()
    {
        return links.back().end;
    }
};

int main()
{
    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(WINDOW_WIDTH, WINDOW_HEIGHT)), "Inverse Kinematics Solver");
    window.setFramerateLimit(60);

    InverseKinematicsSolver solver(NUM_JOINTS);

    while (window.isOpen())
    {
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
        };

        if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left))
        {
            sf::Vector2i mousePosition = sf::Mouse::getPosition(window);
            Eigen::Vector2f target(static_cast<float>(mousePosition.x), static_cast<float>(mousePosition.y));
            solver.setTarget(target);
        }

        solver.update();

        window.clear();
        solver.draw(window);
        window.display();
    }

    return 0;
};

// g++ -std=c++17 -I/opt/homebrew/Cellar/sfml/3.0.0/include -I/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3 -o output inverse.cpp -L/opt/homebrew/Cellar/sfml/3.0.0/lib -lsfml-graphics -lsfml-window -lsfml-system
