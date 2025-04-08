use proc_macro::TokenStream;
#[allow(unused_imports)]
use quote::ToTokens;
#[allow(unused_imports)]
use syn::{parse::Parser, parse_macro_input, DeriveInput};
use quote::quote;

/*
#[proc_macro_attribute]
pub fn spi_device(_args: TokenStream, input: TokenStream) -> TokenStream {
    let mut device_struct = syn::parse_macro_input!(input as syn::ItemStruct);
    device_struct.generics.params.insert(0, syn::parse_quote!(S: ::sirin::spi::SpiConfig));
    match device_struct.fields {
        syn::Fields::Named(ref mut fields) => {
            fields.named.insert(0,
                syn::Field::parse_named.parse2(quote! { spi: ::sirin::spi::Spi<S> }).unwrap()
            );
            fields.named.insert(1,
                syn::Field::parse_named.parse2(quote! { cs_pin: ::embassy_stm32::gpio::AnyPin }).unwrap()
            );
        },
        _ => panic!("SpiDevice must be a struct with named fields"),
    }

    device_struct.into_token_stream().into()
}*/

// Note, this won't work in downstream crates.
#[proc_macro_derive(SpiError)]
pub fn derive_spi_error(item: TokenStream) -> TokenStream {
    let item: DeriveInput = parse_macro_input!(item);
    let name = item.ident;
    let (impl_generics, ty_generics, where_clause) = item.generics.split_for_impl();

    quote! {
        impl #impl_generics embedded_hal_async::spi::ErrorType for #name #ty_generics #where_clause {
            type Error = crate::spi::SpiError;
        }
    }.into()
}

#[proc_macro_derive(Measurement)]
pub fn derive_measurement(item: TokenStream) -> TokenStream {
    let item: DeriveInput = parse_macro_input!(item);
    let name = item.ident;

    let syn::Data::Struct(data) = item.data else {
        return quote! {compile_error!("Only structs allowed")}.into()
    };

    let fields = data.fields.iter().map(|f| f.ident.clone());

    let (impl_generics, ty_generics, where_clause) = item.generics.split_for_impl();

    quote! {
        impl #impl_generics crate::subsystems::Measurement for #name #ty_generics #where_clause {
            fn unmeasured() -> Self {
                Self {
                    #(#fields: Err(crate::subsystems::SubsystemError::NotYetMeasured)),*
                }
            }
        }
    }.into()
}