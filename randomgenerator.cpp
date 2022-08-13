#include <iostream>
#include <algorithm>
#include <random>
#include <vector> 
#include <SFML/Graphics.hpp>

struct test{
    float x;
    float y;
};
int main() {
    /*std::random_device rd;
    std::default_random_engine gen(rd());
    std::uniform_real_distribution<float> dist1(1,500);
    std::uniform_real_distribution<float> dist2(1,500);*/

    test v1{100,200};
    test v2{350, 20};
    test v3{420, 150};
    test v4{80, 380};
    std::vector<test> v(15);
    v.push_back(v1);
    v.push_back(v2);
    v.push_back(v3);
    v.push_back(v4);
    
    //std::generate(v.begin() , v.end(), [&dist1, &dist2, &gen] () ->test {return {dist1(gen), dist2(gen)}; });
    
    auto it = v.begin();
    for (; it != v.end(); ++it) {
        std::cout << it->x << '\t' << it->y << '\n';
    }

    sf::RenderWindow window(sf::VideoMode(500., 500.), "It works");
    window.setFramerateLimit(60);

   
    
    std::vector<sf::RectangleShape> rectangles(4);
    for(; it != v.end(); ++it){
        sf::RectangleShape rect;
        rect.setSize(sf::Vector2f(10.,10.));
        rect.setFillColor(sf::Color::Red);
        rect.setPosition(it->x, it->y);
        rectangles.push_back(rect);
    }
    std::cout<<"Size: " << rectangles.size() <<'\n';
    for(int i = 0; i != 4; i++){
        std::cout<< "x position: " << rectangles[i].getPosition().x <<'\t'<<"y position: " << rectangles[i].getPosition().y << '\n';
    }
    while(window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if(event.type == sf::Event::Closed) window.close();
            if(sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) window.close();
        }

        window.clear();
        window.display();
    }
}
