# Golden data for gnss-m2sdr's subchip replica tests, generated from GNSSSignals.jl.
using GNSSSignals, JSON3, Unitful

const NCHIP = 528          # 33 * 16: a whole number of TMBOC pattern periods
# The replica is sampled on the *gateware's own* phase grid: a frac_bits
# fixed-point code NCO stepping CODE_STEP per sample from phase 0. Phase k is
# then exactly k*CODE_STEP/2^FRAC_BITS chips, with no rounding anywhere, so the
# Python test can compare sample for sample instead of interpolating -- and a
# tap `s` samples early is just golden[k+s], which is how the tap offsets get
# checked against GNSSSignals too.
const FRAC_BITS = 20
const CODE_STEP = 218453        # ~0.20833 chips/sample (4.8 samples/chip)
const NPH   = 3000              # samples of replica per signal (wraps the 528-chip excerpt)

sig(name) = getfield(GNSSSignals, name)()

# LOC has no subcarrier method in GNSSSignals (there is nothing to evaluate); it is
# the unit subcarrier by definition.
sc(::GNSSSignals.LOC, phase) = 1.0
sc(m, phase) = Float64(GNSSSignals.get_subcarrier_code(m, phase))

signals = [
    (:GPSL1CA,          1),
    (:GalileoE1B,       1),
    (:GalileoE1C,       1),
    (:GalileoE1B_BOC11, 1),
    (:GalileoE1C_BOC11, 1),
    (:GPSL1C_D,         1),
    (:GPSL1C_P,         1),
    (:BeiDouB1C_D,      1),
    (:BeiDouB1C_P,      1),
    (:GalileoE1B,       7),
    (:GPSL1C_P,        19),
]

out = Dict{String,Any}()
out["_source"] = "GNSSSignals.jl v" * string(pkgversion(GNSSSignals)) *
    "; primary chips via get_code_at_index (no secondary), replica = primary * get_subcarrier_code"
out["n_chips"] = NCHIP
out["n_samples"]  = NPH
out["frac_bits"]  = FRAC_BITS
out["code_step"]  = CODE_STEP
out["cboc_int_amplitudes"] = collect(GNSSSignals._cboc_int_amplitudes(GNSSSignals.get_modulation(GalileoE1B).boc1_power))

# One chip's worth of subcarrier, per modulation, at every chip position of a
# TMBOC pattern period (33) so the position-dependent variant is covered too.
mods = Dict(
    "LOC"        => GNSSSignals.LOC(),
    "BOCsin_1_1" => GNSSSignals.BOCsin(1, 1),
    "BOCsin_6_1" => GNSSSignals.BOCsin(6, 1),
    "BOCcos_1_1" => GNSSSignals.BOCcos(1, 1),
    "CBOC_E1B"   => GNSSSignals.get_modulation(GalileoE1B),
    "CBOC_E1C"   => GNSSSignals.get_modulation(GalileoE1C),
    "TMBOC_L1CP" => GNSSSignals.get_modulation(GPSL1C_P),
)
subc = Dict{String,Any}()
for (name, m) in mods
    # Only TMBOC's subcarrier depends on the chip position; two positions are
    # enough to pin that the others do not (the tests assert it).
    npos = m isa GNSSSignals.TMBOC ? 33 : 2
    vals = Float64[]
    for pos in 0:(npos-1), k in 0:255
        push!(vals, round(sc(m, pos + k / 256), digits = 9))
    end
    subc[name] = Dict("chip_positions" => npos, "sub_phases" => 256, "values" => vals)
end
out["subcarrier"] = subc

sigs = Dict{String,Any}()
for (name, prn) in signals
    s = sig(name)
    L = get_code_length(s)
    chips = Int[GNSSSignals.get_code_at_index(s, c, prn) > 0 ? 1 : 0 for c in 0:(NCHIP-1)]
    m = GNSSSignals.get_modulation(s)
    # Exact rational phases of the fixed-point NCO, reduced onto the excerpt.
    phases = [mod((i * CODE_STEP) / (1 << FRAC_BITS), NCHIP) for i in 0:(NPH-1)]
    # The gateware replicates the *primary* code only (the host removes the
    # overlay), so the reference is primary chip x subcarrier -- not get_code,
    # which multiplies in secondary chip 0.
    vals = Float64[]
    for p in phases
        c = mod(floor(Int, p), NCHIP)
        push!(vals, round(Float64(GNSSSignals.get_code_at_index(s, c, prn)) * sc(m, p),
                          digits = 9))
    end
    sigs["$(name)_prn$(prn)"] = Dict(
        "signal" => String(name), "prn" => prn,
        "code_length" => L,
        "code_frequency" => Float64(ustrip(get_code_frequency(s))),
        "modulation" => string(nameof(typeof(m))),
        "subchip_factor" => (m isa GNSSSignals.LOC ? 1 :
                             m isa GNSSSignals.CBOC ? 2 * lcm(m.boc1.m, m.boc2.m) :
                             m isa GNSSSignals.TMBOC ? 2 * m.boc2.m :
                             m isa GNSSSignals.BOCcos ? 4 * m.m : 2 * m.m),
        "code_amplitude" => Float64(GNSSSignals.get_code_amplitude(s)),
        "secondary_code_length" => Int(GNSSSignals.get_secondary_code_length(s)),
        "chips" => join(chips),
        "replica" => vals,
    )
end
out["signals"] = sigs
out["tmboc_pattern"] = collect(Int.(GNSSSignals.get_modulation(GPSL1C_P).pattern))

open("l1_subchip_golden.json", "w") do io
    JSON3.write(io, out)
end
println("written ", filesize("l1_subchip_golden.json"), " bytes")
