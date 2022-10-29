//#![no_std]

extern crate proc_macro;
use proc_macro::TokenStream;
use quote::quote;
use std::vec;
use syn::{parse_macro_input, Ident};

fn bit_4_u32(v: bool) -> u32 {
    // awkward pattern to FORCE highest bit to always be 0 because
    // STM32 will have MOSI idle state equal to MSB of one of the bytes:
    //
    // https://michaeltien8901.github.io/stm32/2019/01/06/STM32F072-MOSI-Idle-State.html
    if v {
        0b0111 as u32
    } else {
        0b0100 as u32
    }
}

fn nibble_12_u32(nibble: u8) -> u32 {
    (bit_4_u32((nibble & 0b1000) != 0) << 12)
        | (bit_4_u32((nibble & 0b0100) != 0) << 8)
        | (bit_4_u32((nibble & 0b0010) != 0) << 4)
        | (bit_4_u32((nibble & 0b0001) != 0) << 0)
}

#[proc_macro]
pub fn ws2812_constants(input: TokenStream) -> TokenStream {
    let ident = parse_macro_input!(input as Ident);

    let mut values = vec![];

    for v in 0..=0xFFu8 {
        let data = (nibble_12_u32((v >> 4) & 0x0F) << 16) | nibble_12_u32(v & 0x0F);

        values.push([
            ((data >> 24) & 0xFF) as u8,
            ((data >> 16) & 0xFF) as u8,
            ((data >> 8) & 0xFF) as u8,
            (data & 0xFF) as u8,
        ]);
    }

    let values = values.iter().map(|[a, b, c, d]| {
        quote! {
            [#a, #b, #c, #d ]
        }
    });

    quote! {
        const #ident : [[u8;4]; 256] = [
          #(#values), *
        ];
    }
    .into()
}
