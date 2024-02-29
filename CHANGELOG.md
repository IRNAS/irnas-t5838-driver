# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

## [Unreleased]

## [0.1.1] - 2024-02-29

### Fixed

-   Remove `example_lib` (and by extension `tests/sample_test`), as it was causing 
    linker errors in the projects that are importing `irnas-t5838-driver` project 
    and also have `example_lib` in it.

## [0.1.0] - 2023-11-07

### Added

-   Initial release
-   Added sample for microphone triggered sampling

[Unreleased]: https://github.com/IRNAS/irnas-t5838-driver/compare/v0.1.1...HEAD

[0.1.1]: https://github.com/IRNAS/irnas-t5838-driver/compare/v0.1.0...v0.1.1

[0.1.0]: https://github.com/IRNAS/irnas-t5838-driver/compare/02156306967d34d38bde29ffc9f8eea1edccc8d7...v0.1.0
