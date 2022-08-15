// Compile with: g++ -Wall -Wextra -fsanitize=address boids.cpp rulesofflight.cpp main.cpp -lsfml-graphics -lsfml-window -lsfml-system 

//close the window from sfml button
#include "boids.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <iostream>
#include <random>
#include <cstdlib>


auto evolve(Boids& boids, int spevolution, sf::Time delta_t) {
  double const unitoft{delta_t.asSeconds()};
  for (int i{0}; i != spevolution; ++i) { // attenzione!!!! potrebbe esserci problema di velocità
    boids.evolution(unitoft);
  }
  return boids.TotalBoids();
}

int main() {
  std::random_device rd;
  std::default_random_engine gen(rd());
  std::uniform_real_distribution<double> dist(0, 690);
  std::uniform_real_distribution<double> dist2(-100, 100);

  std::cout << "Insert number of boids (at least 2): " << '\n';
  int n;
  std::cin >> n;
  if(std::cin.fail()|| n<=1){ 
    std::cerr << "Invalid number\n";
    return EXIT_FAILURE;
  }
  std::cout
      << "Insert separation const, allignement const ( < 1 ), cohesion const: "
      << '\n';  // valori di n ottimali: intorno a 20 per il momento
  double s;
  double a;
  double c;

  std::cin >> s >> a >> c;
  if(std::cin.fail()|| a>1){
    std::cerr << "Invalid number\n";
    return EXIT_FAILURE;
  }

  Boids boids{n, 400., SeparationRule{s, 250.}, AllignmentRule{a},
            CohesionRule{c}};

  auto vec_boids = boids.TotalBoids();

  vec_boids.resize(n);
  
  std::generate(vec_boids.begin(), vec_boids.end(), [&gen, &dist, &dist2]() -> BoidState {
    return {dist(gen), dist(gen), dist2(gen), dist2(gen)};
  });
  // std::cout << bob1[1].x <<'\n';

  boids.setvector(vec_boids);

  state(boids, 0.1);

  // std::cout << (bob.TotalBoids())[1].x << '\n';


  auto const delta_t{sf::milliseconds(1)};
  int const fps = 30;
  int const step_evolution{300 / fps};

 

  sf::RenderWindow window(sf::VideoMode(1179, 691), "sperem");
  window.setFramerateLimit(fps);
  sf::Texture texture;
  if (!texture.loadFromFile("sfondomare.png")) {
    // sollevare un'eccezione eventualmente
  }
  sf::Sprite sprite;
  sprite.setTexture(texture);

  sf::CircleShape triangle;
  triangle.setRadius(10.f);
  triangle.setPointCount(3);

  triangle.setFillColor(sf::Color(245, 152, 66));

  while (window.isOpen())  {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }
    auto boidscopy = evolve(boids, step_evolution, delta_t);
    int i = 0;

    window.clear();
    window.draw(sprite);
    for(auto& b : boidscopy){
      triangle.setPosition(b.x, b.y);
      switch(static_cast<int>(b.y)) {
        case 0:
        triangle.move(0.f, 691); 
        boids.TotalBoids()[i].y = 691;      
        break;
        case 691:
        triangle.move(0.f, -691);
        boids.TotalBoids()[i].y = 0.; 
        break;
        ++i;
      }  
    
      window.draw(triangle);
  
    }


    window.display();
  }

}
