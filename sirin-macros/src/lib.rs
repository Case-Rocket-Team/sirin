#![allow(unused_imports)]
use std::collections::HashMap;

use proc_macro::TokenStream;
use quote::ToTokens;
use syn::{parse::{self, Parse}, punctuated::Punctuated, spanned::Spanned, token::Comma, Attribute, DataEnum, DataStruct, Error, Field, Fields, Ident, ItemEnum, Meta, Token, Variant, Visibility};

use syn::{parse::Parser, parse_macro_input, DeriveInput};
use quote::quote;
use proc_macro2::{Delimiter, Span, TokenStream as TokenStream2, TokenTree};
use anyhow::bail;

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

#[proc_macro_derive(SongSize, attributes(song))]
pub fn derive_song_size(tok: TokenStream) -> TokenStream {
    let tok1 = tok.clone();
    let item: DeriveInput = parse_macro_input!(tok1);
    let ident = item.ident;

    match item.data {
        syn::Data::Struct(s) => derive_song_size_struct(ident, s).unwrap().into(),
        syn::Data::Enum(e) => {
            let enum_song: EnumSong = parse_macro_input!(tok);
            let tok = derive_song_size_enum(&enum_song).unwrap();
            //println!("{}", tok.to_string());
            tok.into()
        },
        _ => todo!()
    }
}


#[proc_macro_derive(ToSong, attributes(song))]
pub fn derive_to_song(tok: TokenStream) -> TokenStream {
    let tok1 = tok.clone();
    let item: DeriveInput = parse_macro_input!(tok1);
    let ident = item.ident;

    match item.data {
        syn::Data::Struct(s) => derive_to_song_struct(ident, s).unwrap().into(),
        syn::Data::Enum(e) => {
            let enum_song: EnumSong = parse_macro_input!(tok);
            let tok = derive_to_song_enum(&enum_song).unwrap();
            //println!("{}", tok.to_string());
            tok.into()
        },
        _ => todo!()
    }
}

struct EnumSong {
    item: ItemEnum,
    disc_name: Ident,
    disc_type: Ident,
}

impl Parse for EnumSong {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let item: ItemEnum = input.parse()?;

        let attrs: Vec<TokenStream2> = item.attrs.iter().filter_map(|a| {
            if a.path().get_ident()?.to_string() != "song" {
                return None;
            }
    
            let Meta::List(ref list) = a.meta else {
                return None
            };
    
            Some(list.tokens.clone())
        }).collect();

        if attrs.len() != 1 {
            return Err(Error::new(
                attrs.get(0).map(|i| i.span()).unwrap_or_else(|| item.span()),
                "Expected 1 attr -- discriminant"
            ));
        }

        let mut attr = attrs[0].clone().into_iter();

        let disc_ident: Ident = syn::parse2(attr.next().unwrap().to_token_stream())?;
        if disc_ident.to_string() != "discriminant" {
            return Err(Error::new_spanned(disc_ident, "Expected 'discriminant'"));
        }

        let group: proc_macro2::Group = syn::parse2(attr.next().unwrap().to_token_stream())?;
        if group.delimiter() != Delimiter::Parenthesis {
            return Err(Error::new(group.delim_span().span(), "Expected parens"));
        }

        let mut iter = group.stream().into_token_stream().into_iter();
        let disc_name: Ident = syn::parse2(iter.next().unwrap().into_token_stream())?;
        let _: Token![=] = syn::parse2(iter.next().unwrap().into_token_stream())?;
        let disc_type: Ident = syn::parse2(iter.next().unwrap().into_token_stream())?;

        /*if disc_type.to_string() != "u8" {
            return Err(Error::new(disc_type.span(), "Only u8 supported rn"));
        }*/

        Ok(EnumSong {
            item,
            disc_name,
            disc_type,
        })
    }
}

fn derive_song_size_struct(ident: Ident, item: DataStruct) -> Result<TokenStream2, anyhow::Error> {
    let mut size_out = vec![];

    for field in item.fields {
        let Field { ident, .. } = field;

        size_out.push(quote! {
            self.#ident.song_size()
        });
    }
    
    Ok(quote! {
        impl SongSize for #ident {
            fn song_size(&self) -> usize {
                0 #( + #size_out)*
            }
        }
    })
}

fn derive_to_song_struct(ident: Ident, item: DataStruct) -> Result<TokenStream2, anyhow::Error> {
    let mut fields_out = vec![];

    for field in item.fields {
        let ident = field.ident;

        fields_out.push(quote! {
            self.#ident.to_song(&mut buf[i..])?;
            i += self.#ident.song_size();
        });
    }

    Ok(quote! {
        impl ToSong for #ident {
            fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
                let size = self.song_size();
                if buf.len() >= size {
                    let mut i = 0;
                    #(#fields_out)*
                    Ok(())
                } else {
                    Err(ToSongError::BufferOverflow)
                }
            }
        }
    })
}

fn derive_to_song_enum(enum_song: &EnumSong) -> syn::Result<TokenStream2> {
    let ident = &enum_song.item.ident;
    let desc_ident = &enum_song.disc_name;
    let desc_ty = &enum_song.disc_type;

    let mut out = vec![];

    for var in &enum_song.item.variants {
        let mut idents = vec![];

        let mut i = 0;
        for field in &var.fields {
            // println!("Debug 100: {:?}", field);
            idents.push(
                field.ident.clone().unwrap_or_else(
                    || Ident::new(&format!("t{}", i).to_string(), field.span())
                )
            );
            i += 1;
        }

        let mut fields_out = vec![];

        for ident in &idents {
            fields_out.push(quote! {
                #ident.to_song(&mut buf[i..])?;
                i += #ident.song_size();
            });
        }

        let ident = &var.ident;

        let destructure = match &var.fields {
            Fields::Unit => quote!(),
            Fields::Named(_) => quote!({ #(#idents),* }),
            Fields::Unnamed(_) => quote!(( #(#idents),* ))
        };

        out.push(quote! {
            Self::#ident #destructure => {
                (#desc_ident::#ident as #desc_ty).to_song(buf)?;
                let mut i = core::mem::size_of::<#desc_ty>();
                #(#fields_out)*
                Ok(())
            }
        });
    }
    
    Ok(quote! {
        impl ToSong for #ident {
            fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
                match self {
                    #(#out),*
                }
            }
        }
    })
}

fn derive_song_size_enum(enum_song: &EnumSong) -> syn::Result<TokenStream2> {
    let ident = &enum_song.item.ident;
    let disc_type = &enum_song.disc_type;

    let mut out = vec![];

    let mut disc_out = vec![];

    for var in &enum_song.item.variants {
        let disc_var_ident = &var.ident;

        match &var.discriminant {
            Some((eq, val)) => {
                disc_out.push(quote! {
                    #disc_var_ident #eq #val
                });
            }
            None => {
                disc_out.push(quote! {
                    #disc_var_ident
                });
            }
        }

        let mut idents = vec![];

        let mut i = 0;
        for field in &var.fields {
            idents.push(
                field.ident.clone().unwrap_or_else(
                    || Ident::new(&format!("t{}", i).to_string(), field.span())
                )
            );
            i += 1;
        }

        let mut fields_out = vec![];

        for ident in &idents {
            fields_out.push(quote! {
                i += #ident.song_size();
            });
        }

        let ident = &var.ident;

        let destructure = match &var.fields {
            Fields::Unit => quote!(),
            Fields::Named(_) => quote!({ #(#idents),* }),
            Fields::Unnamed(_) => quote!(( #(#idents),* ))
        };

        out.push(quote! {
            Self::#ident #destructure => {
                let mut i = core::mem::size_of::<#disc_type>();
                #(#fields_out)*
                i
            }
        });
    }

    let disc_ident = &enum_song.disc_name;
    let disc_type = &enum_song.disc_type;
    let vis = &enum_song.item.vis;

    Ok(quote! {
        #[repr(#disc_type)]
        #vis enum #disc_ident {
            #(#disc_out),*
        }

        impl SongSize for #ident {
            fn song_size(&self) -> usize {
                match self {
                    #(#out),*
                }
            }
        }
    })
}

#[proc_macro_derive(FromSong, attributes(song))]
pub fn derive_from_song(tok: TokenStream) -> TokenStream {
    let tok1 = tok.clone();
    let item: DeriveInput = parse_macro_input!(tok1);
    let ident = item.ident;

    match item.data {
        syn::Data::Struct(s) => derive_from_song_struct(ident, s).unwrap().into(),
        syn::Data::Enum(e) => {
            let enum_song: EnumSong = parse_macro_input!(tok);
            let tok = derive_from_song_enum(&enum_song).unwrap();
            // println!("{}", tok.to_string());
            tok.into()
        },
        _ => todo!()
    }
}

fn derive_from_song_struct(ident: Ident, item: DataStruct) -> Result<TokenStream2, anyhow::Error> {
    let mut from_song_out = vec![];
    
    for field in item.fields {
        let Field { ident, ty, .. } = field;

        from_song_out.push(quote! {
            #ident: {
                let value = <#ty as FromSong>::from_song(&buf[i..])?;
                i += value.song_size();
                value
            }
        });
    }

    Ok(quote! {
        impl FromSong for #ident {
            fn from_song(buf: &[u8]) -> Result<Self, FromSongError> {
                let mut i = 0;
                Ok(
                #ident {
                    #(#from_song_out,)*
                }
                )
            }
        }
    })
}


fn derive_from_song_enum(enum_song: &EnumSong) -> syn::Result<TokenStream2> {
    let ident1 = &enum_song.item.ident;
    let desc_ident = &enum_song.disc_name;
    // let item = &enum_song.item;

    let mut out = vec![];
    let mut k = 0;
    for var in &enum_song.item.variants {

        let mut fields_out = vec![];
        let mut idents = vec![];
        
        let mut i = 0;
        for field in &var.fields {
            let ident = field.ident.clone().unwrap_or_else(
                || Ident::new(&format!("t{}", i).to_string(), field.span()));
            idents.push(ident.clone());
            
            let typ = field.ty.clone();
            
            fields_out.push(quote! {
                let #ident = #typ::from_song(&buf[i..])?;
                i += #ident.song_size();
            });
            i += 1;
        }   

        let ident = &var.ident;


        match &var.fields {
            Fields::Unit => out.push(quote! {
                    val if val == #desc_ident::#ident as u8 => Ok(#ident1::#ident)
                }),
            Fields::Unnamed(_) => out.push(quote! { 
                val if val == #desc_ident::#ident as u8 => {
                    let mut i = 1;
                    #(#fields_out)*
                    Ok(#ident1::#ident(#(#idents,)*))
                }}),
            Fields::Named(_) => todo!()

        };

        // println!("Debug 22: {:?}", out[k].to_string());
        k += 1;
    }

    // let test = quote! {
    //     impl FromSong for #ident1 {
    //         fn from_song(buf: &[u8]) -> Result<Self, FromSongError> {
    //             let Some(&disc) = buf.get(0) else {
    //                 return Err(FromSongError::BufferOverflow)
    //             };

    //             match disc {
    //                 #(#out,)*
    //                 _ => Err(FromSongError::InvalidPacketId),
    //             }
    //         }
    //     }
    // };

    // println!("Debug 3: {}", test.to_string());
    
    Ok(quote! {
        impl FromSong for #ident1 {
            fn from_song(buf: &[u8]) -> Result<Self, FromSongError> {
                let Some(&disc) = buf.get(0) else {
                    return Err(FromSongError::BufferOverflow)
                };

                match disc {
                    #(#out,)*
                    _ => Err(FromSongError::InvalidPacketId)
                }
            }
        }
    })
}