# light-valves
A repository holding firmware and documentation for driving many polarized LCD squares, aka light valves, at once.

Directory structure:
* light-valve-node: firmware and test files for what runs on each individual LCD driver PCBA, one board per light valve
* light-valve-driver: firmware and test files for what runs on the driver board responsible for receiving data over RS-485 and broadcasting instructions for up to 16 nodes on its data bus
* light-valve-touchdesigner: TD project for driving nodes with custom serial protocol based on implemented patterns
* doc - all documentation and scattered files for each section of the project
