## Practical Challenges and Experience

During this project, one of the main difficulties was dealing with uneven motor and wheel response.  
In practice, the robot did not always move as expected: straight movement was not always stable, turning behavior could be unbalanced, and startup could sometimes be too aggressive before becoming less smooth.

A large part of the work involved repeated parameter tuning and real testing.  
This helped improve my understanding of how theoretical control differs from real hardware behavior, especially when motor response, wheel contact, and mechanical conditions are not perfectly consistent.

Another important aspect of the project was the coordination between Arduino Mega and ESP32.  
The system was not based on a single board, so communication between motion control and Wi-Fi control had to be considered carefully.  
I also learned that wireless control, motion response, and hardware tuning must work together in a practical embedded system.

The project was also influenced by hardware constraints.  
Available components, power distribution, voltage regulation, and mechanical structure all had an effect on the final performance.  
This gave me more practical experience in debugging real systems under limited conditions, rather than only working in an ideal setup.

In addition, I faced some development environment and board compatibility issues during related embedded work, which helped me become more familiar with practical setup, adaptation, and troubleshooting across different platforms.
