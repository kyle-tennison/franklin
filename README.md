# franklin
A balancing robot

![](./cad-screenshot.png)


## Background

Franklin is a little balancing robot that I'm working on in my free time. He isn't meant to do anything spectacular; I  thought it would be a good project to help me get a bit better with electronics and low-level programming.

Why the name Franklin? I'm not sure. It was funny to me in the moment, and it kind of stuck.

## How Does He Work?

### Communication

The ESP hosts an access point that we connect to via a Rust client. We establish a socket connection and communicate via a low-level TCP interface.

More documentation coming soon.
