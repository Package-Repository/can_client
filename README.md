# Service Apis

Generic ROS2 Client API to help make service calls
Basically to integrate with any service we write, you would have to figure out how to properly make
the service call and also pass the appropriate parameters. Instead, each service can be shipped with 
the appropriate client code so someone can use one public function to integrate with another package.
Every package that needs to interface with another module could import this central module

## Usage

Clone this repo and run the install.sh script to place all python files in a pre-determined directory, this way there are no issues importing code
Each file has the interpreter placed at the top, so we can run the scripts by typing in the name.

@Strix Elixel[https://github.com/Repo-Factory]