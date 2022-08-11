#include "boids.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>

auto evolution(Boids& pesci, int steps_per_evolution, sf::Time delta_t)
{
  double const dt{delta_t.asSeconds()};

  for (int i{0}; i != steps_per_evolution; ++i) {
    pesci.evolution(dt);
  }

  return pesci.state();
}

int main () {
    //something
    auto const delta_t{sf::milliseconds(1)};
    int const fps = 30;
    int const steps_per_evolution{1000/fps};

    unsigned const display_width = .9 * sf::VideoMode::getDesktopMode().width;
    unsigned const display_height = .9 * sf::VideoMode::getDesktopMode().height;
    sf::RenderWindow window(sf::VideoMode(display_width, display_height), "Mare");
    window.setFramerateLimit(fps);
    sf::Texture texture;
    if(!texture.loadFromFile("sfondomare.png"))
    {
        //da sollevare un'eccezione eventualmente
    } 
    sf::Sprite sprite;
    sprite.setTexture(texture);

    sf::CircleShape fish;
    fish.setRadius(8.0f);
    fish.setPointCount(3);
    fish.setFillColor(sf::Color::Magenta);

    while(window.isOpen()) {
        sf::Event event;
        while(window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {window.close(); }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) {window.close(); }
        }
    window.clear();
    window.draw(sprite);
    fish.setPosition(500, 400);
    window.draw(fish);
    window.display();
    }
}