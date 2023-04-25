
# draca_planner

  

## About



draca_planner (Deep Reinforcement learning based Collision Avoidance) is designed to provide a deep reinforcement learning-based collision avoidance solution for drone swarms using PyTorch models. It is built for the [mrs_uav_system](https://github.com/ctu-mrs/mrs_uav_system) and is intended to accelerate the **testing and development of deep reinforcement learning algorithms for motion planning.**

The package includes a trained PyTorch model that can control drone swarms to avoid collisions in a simulated environment. The model is trained using a custom [OCRA](https://gamma.cs.unc.edu/ORCA/) dataset generated using [RVO2](https://gamma.cs.unc.edu/RVO2/) and is further improved using reinforcement learning.

With `draca_planner`, users can easily incorporate deep reinforcement learning algorithms into their drone swarm motion planning pipeline. This package offers an efficient and effective solution for developers who seek to accelerate the testing and development of their motion planning algorithms.

  


  

## Setting up and running the project

  The project uses the *swarm_control_manager* which is currently under testing and to be open sourced soon. Till then, please head to draca_ros to install and test the package as a **singularity container**.


  


## Support

If you encounter any bugs, please raise a GitHub issue. For suggestions or other forms of support, please contact us at [vinayakkapoor12@gmail.com](mailto:vinayakkapoor12@gmail.com).

## Authors and Acknowledgments

I would like to thank [Dr. Martin Saska](http://mrs.felk.cvut.cz/people/martin-saska) and the entire research group for their constant help. I am eternally grateful to [Mr. Afzal Ahmad](http://mrs.felk.cvut.cz/people/afzal-ahmad) and [Dr. Daniel Bonilla Licea](http://mrs.felk.cvut.cz/members/postdocs/daniel-bonilla) for guiding me at every step of the way. It was indeed a pleasure to learn under their able guidance. I would also like to extend my gratitude to [Dr. Jitendra Singh Rathore](https://www.bits-pilani.ac.in/pilani/jitendrarathore/profile) for his support and encouragement throughout the project.

## License

draca_planner is released under the MIT License.

## Project Status

draca_planner is an actively maintained project, and we welcome feedback and contributions from the community. Please let us know if you encounter any bugs or issues.
