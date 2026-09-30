"""Name derivation for the scaffolder.

Pure functions, no I/O. Every naming decision the generator makes lives here so
it can be reasoned about (and tested) in one place.

The rules exist because this repo's conventions are inconsistent:
  - include guards split 130 `#ifndef X_HPP_` vs 44 `#pragma once` -> guards win
  - directories may be camelCase (`clientDisplay`) while their namespace is
    snake_case (`src::control::client_display`), so a namespace segment is never
    just the literal directory name
"""

import re

# Matches, in priority order: a run of capitals not followed by a lowercase
# letter (acronyms), a capitalized word, or a run of lowercase/digits.
#   "DJITwoFlywheel" -> dji, two, flywheel
#   "UISubsystem"    -> ui, subsystem
_TOKEN = re.compile(r"[A-Z]+(?![a-z])|[A-Z][a-z0-9]*|[a-z0-9]+")

# Enough of the keyword list to catch a plausible mistake. Not exhaustive by
# design -- the compiler is the real backstop.
_CPP_KEYWORDS = {
    "alignas", "alignof", "and", "asm", "auto", "bool", "break", "case", "catch",
    "char", "class", "const", "constexpr", "continue", "decltype", "default",
    "delete", "do", "double", "else", "enum", "explicit", "export", "extern",
    "false", "float", "for", "friend", "goto", "if", "inline", "int", "long",
    "mutable", "namespace", "new", "noexcept", "not", "nullptr", "operator",
    "or", "private", "protected", "public", "register", "return", "short",
    "signed", "sizeof", "static", "struct", "switch", "template", "this",
    "throw", "true", "try", "typedef", "typename", "union", "unsigned", "using",
    "virtual", "void", "volatile", "while", "xor",
}


class NameError_(ValueError):
    """Raised when an input cannot produce a valid C++ identifier."""


def tokenize(text):
    """Split an arbitrary human-typed name into lowercase word tokens.

    Handles spaces, underscores, hyphens, camelCase, PascalCase and acronyms.
    """
    tokens = []
    for chunk in re.split(r"[^A-Za-z0-9]+", text):
        if chunk:
            tokens.extend(m.group(0).lower() for m in _TOKEN.finditer(chunk))
    return tokens


def pascal(tokens):
    return "".join(t.capitalize() for t in tokens)


def lower_camel(tokens):
    if not tokens:
        return ""
    return tokens[0] + "".join(t.capitalize() for t in tokens[1:])


def snake(text):
    return "_".join(tokenize(text))


def namespace_for(directory):
    """`src/control/<dir>` -> `src::control::<snake dir>`.

    Each path segment is snake-cased independently, so `clientDisplay` becomes
    `client_display` -- matching what the existing code actually does.
    """
    segments = [snake(seg) for seg in directory.strip("/").split("/") if seg]
    if not segments:
        raise NameError_("feature directory is empty")
    return "::".join(["src", "control"] + segments)


def derive(
    raw_name,
    kind,
    directory,
    class_override=None,
    instance_override=None,
    namespace_override=None,
):
    """Derive every name the templates need.

    `kind` is "subsystem" or "command". A trailing token matching `kind` is
    stripped from the input so `Flywheel`, `flywheel_subsystem` and
    `FlywheelSubsystem` all collapse to the same result -- never double-suffixed.
    """
    if kind not in ("subsystem", "command"):
        raise NameError_("kind must be 'subsystem' or 'command'")

    tokens = tokenize(raw_name)
    if not tokens:
        raise NameError_("name %r contains no letters or digits" % raw_name)

    if tokens[-1] == kind:
        tokens = tokens[:-1]
    if not tokens:
        raise NameError_("name %r is only the word %r" % (raw_name, kind))

    if tokens[0][0].isdigit():
        raise NameError_("name %r starts with a digit" % raw_name)

    stem = "_".join(tokens + [kind])
    class_name = class_override or (pascal(tokens) + kind.capitalize())
    instance = instance_override or lower_camel(tokenize(class_name))

    for ident, what in ((class_name, "class name"), (instance, "instance name")):
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", ident):
            raise NameError_("derived %s %r is not a valid identifier" % (what, ident))
        if ident in _CPP_KEYWORDS:
            raise NameError_("derived %s %r is a C++ keyword" % (what, ident))

    ns = namespace_override or namespace_for(directory)
    for segment in ns.split("::"):
        if segment in _CPP_KEYWORDS:
            raise NameError_(
                "directory %r produces namespace segment %r, which is a C++ keyword"
                % (directory, segment)
            )

    return {
        "kind": kind,
        "tokens": tokens,
        "stem": stem,
        "class_name": class_name,
        "instance": instance,
        "guard": stem.upper() + "_HPP_",
        "namespace": ns,
        "directory": directory.strip("/"),
        "display_name": " ".join(tokens + [kind]),
        "include": "control/%s/%s.hpp" % (directory.strip("/"), stem),
    }
