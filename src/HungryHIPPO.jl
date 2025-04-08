module HungryHIPPO

    include("WaveFrontPlanner.jl")
    include("WaveFrontGen.jl")

    export
    wavefrontPlanner,
    get_direct_path

end
