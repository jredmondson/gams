workspace(name = "gams")

http_archive(
    name = "madara",
    strip_prefix = "madara-a05f82a1b72f0facc51a783c2f890988468db197",
    urls = ["https://gitlab.com/shield-ai/public/madara/-/archive/a05f82a1b72f0facc51a783c2f890988468db197/madara-a05f82a1b72f0facc51a783c2f890988468db197.tar.gz"],
)

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

git_repository(
    name = "ace",
    remote = "https://gitlab.com/shield-ai/public/ace.git",
    tag = "stable",
)

git_repository(
    name = "bazel_module",
    remote = "https://gitlab.com/shield-ai/public/bazel_module.git",
    tag = "stable",
)

load("@bazel_module//bazel_rules:generate_modules.bzl", "generate_modules")

generate_modules(
    root_path = __workspace_dir__,
)

load("@bazel_module//bazel_rules:module.bzl", "add_modules")

add_modules()
