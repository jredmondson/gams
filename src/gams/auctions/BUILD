package(default_visibility = ["//visibility:public"])

cc_library(
    name = "auctions",
    srcs = glob([
        "*.cpp",
        "*.h",
    ]),
    hdrs = [
        "AuctionBase.h",
        "AuctionBase.inl",
        "AuctionBid.h",
        "AuctionBid.inl",
        "AuctionFactory.h",
        "AuctionFactoryRepository.h",
        "AuctionFactoryRepository.inl",
        "AuctionMaximumBid.h",
        "AuctionMinimumBid.h",
        "AuctionMinimumDistance.h",
        "AuctionTypesEnum.h",
    ],
    include_prefix = "gams/auctions",
    textual_hdrs = [
        "AuctionBase.inl",
        "AuctionBid.inl",
        "AuctionFactoryRepository.inl",
    ],
    deps = [
        "@gams//:gams_base",
        "@gams//src/gams/groups",
        "@gams//src/gams/platforms",
        "@gams//src/gams/utility",
        "@madara",
    ],
)
