
# Fast Verlet Rope Library

A high-performance, cross-platform rope simulation library using the Verlet integration method. For when you need rope, and you need it quickly.

![Screenshot](screenshot.png)

## Features

- Fast and stable rope physics using Verlet integration.
- Supports multiple rope types (single, multi-ply, chain, etc.)
- Configurable rope length, stiffness, and segment count.
- ~~Collision detection and response~~
- Endcap and attachment support for rope details.
- Demo scene and example textures and meshes included.

## Installation

1. Clone the repository:
	```sh
	git clone https://github.com/mikest/gd-fast-rope.git
	```
2. Build the library using SCons or CMake:
	```sh
	scons compiledb=yes
	```
    or for debug builds...
	```sh
	scons dev_build=yes compiledb=yes template_debug=yes
	```
3. Add the compiled `./demo/bin` binaries to your project.

## Usage

1. Import the library into your project.
2. Create a rope instance and configure its parameters.
3. Attach rope ends to objects or positions. Rope details will be scaled to rope width, so model them at 1:1 for a 1m thick rope.

## Demo

See the `demo/` folder for example scenes and usage.

## Documentation

- API reference: See `doc_classes/`
- Example scripts: See `demo/example.gd`
- Build instructions: See `SConstruct`

## Contributing

Pull requests and issues are welcome! Please see `CONTRIBUTING.md` for guidelines.

## License

This project is licensed under the MIT License. See `LICENSE.md` for details.