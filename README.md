# To launch a ROS2 web-bash console

1. [Install Docker](https://docs.docker.com/install/)
1. If you are NOT using Mac, [install Docker Compose (ie docker-compose)](https://docs.docker.com/compose/install/)
1. 'cd docker'
1. 'docker-compose up -d'
1. Open a web browser tab and go to http://localhost:9000 to open Portainer management web page.
   1. (NOTE) If a bunch of red "Failure" tooltips pop up, hit Ctrl-Shift-r to flush cache and refresh URL.
   1. Create a local Portainer admin account
   1. Upon seeing 'Connect Portainer to the Docker environment you want to manage.'
      1. Select the 'Local - manage the local Docker environment' box.
      1. Click 'Connect'
1. Get full container ID for the ros2_console container, from your terminal, type:
   1. 'docker ps --no-trunc -f name=ros2_console -q'
1. Open a new browser tab to launch a web-bash terminal via http://localhost:9000/#/containers/<ROS2_CONSOLE_CID>/exec - replace ROS2_CONSOLE_CID with the 'full container ID' you discovered above.
   1. Click 'Connect'
   
You now have a web-bash terminal into one of the Docker containers we launched (via 'docker-compose up -d') to start learning ROS2. 

If you are reading one of the https://blog.rosdroid.com blog posts, you can use this web-bash to follow along with the instructions described in the blog post.
